/*
 *  drivers/cpufreq/cpufreq_ragingeagles.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Alexander Clouter <alex@digriz.org.uk>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#include <trace/events/cpufreq_ragingeagles.h>

static int g_count = 0;

#define DEF_SAMPLING_RATE                       (15000)
#define DEF_FREQUENCY_UP_THRESHOLD		(25)
#define DEF_FREQUENCY_DOWN_DIFFERENTIAL         (10)

#define MIN_SAMPLING_RATE_RATIO			(2)

static unsigned int min_sampling_rate;

#define MAX_FREQUENCY_THRESHOLD 		(100)
#define MIN_FREQUENCY_THRESHOLD			(10)
#define LATENCY_MULTIPLIER			(1000)
#define MIN_LATENCY_MULTIPLIER			(100)
#define TRANSITION_LATENCY_LIMIT		(10 * 1000 * 1000)
#define POWERSAVE_BIAS_MAXLEVEL                 (1000)
#define POWERSAVE_BIAS_MINLEVEL                 (-1000)

static void do_dbs_timer(struct work_struct *work);

struct cpu_dbs_info_s {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	unsigned int down_skip;
	unsigned int requested_freq;
	int cpu;
	unsigned int enable:1;
	struct mutex timer_mutex;

#ifndef CONFIG_ARCH_MSM_CORTEXMP
        struct task_struct *sync_thread;
        wait_queue_head_t sync_wq;
        atomic_t src_sync_cpu;
        atomic_t sync_enabled;
#endif
};
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cs_cpu_dbs_info);

static unsigned int dbs_enable;	

static DEFINE_MUTEX(dbs_mutex);

static struct workqueue_struct *dbs_wq;

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int up_threshold;
	unsigned int shortcut;	
	unsigned int ignore_nice;
	unsigned int freq_step;
	int gboost;

} dbs_tuners_ins = {
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
        .up_threshold_multi_core = DEF_FREQUENCY_UP_THRESHOLD,
        .down_differential = DEF_FREQUENCY_DOWN_DIFFERENTIAL,
        .up_threshold_any_cpu_load = DEF_FREQUENCY_UP_THRESHOLD,
	.ignore_nice = 1,
        .powersave_bias = 0,
	.freq_step = 5,
	.shortcut = 0,
	.gboost = 1,
};

static inline u64 get_cpu_idle_time_jiffy(unsigned int cpu, u64 *wall)
{
	u64 idle_time;
	u64 cur_wall_time;
	u64 busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

	busy_time  = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = jiffies_to_usecs(cur_wall_time);

	return jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);
	else
		idle_time += get_cpu_iowait_time_us(cpu, wall);

	return idle_time;
}

static unsigned int powersave_bias_target(struct cpufreq_policy *policy,
                                          unsigned int freq_next,
                                          unsigned int relation)
{
        unsigned int freq_req, freq_avg;
        unsigned int freq_hi, freq_lo;
        unsigned int index = 0;
        unsigned int jiffies_total, jiffies_hi, jiffies_lo;
        int freq_reduc;
        struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info,
                                                   policy->cpu);

        if (!dbs_info->freq_table) {
                dbs_info->freq_lo = 0;
                dbs_info->freq_lo_jiffies = 0;
                return freq_next;
        }

        cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_next,
                        relation, &index);
        freq_req = dbs_info->freq_table[index].frequency;
        freq_reduc = freq_req * dbs_tuners_ins.powersave_bias / 1000;
        freq_avg = freq_req - freq_reduc;

        
        index = 0;
        cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
                        CPUFREQ_RELATION_H, &index);
        freq_lo = dbs_info->freq_table[index].frequency;
        index = 0;
        cpufreq_frequency_table_target(policy, dbs_info->freq_table, freq_avg,
                        CPUFREQ_RELATION_L, &index);
        freq_hi = dbs_info->freq_table[index].frequency;

        
        if (freq_hi == freq_lo) {
                dbs_info->freq_lo = 0;
                dbs_info->freq_lo_jiffies = 0;
                return freq_lo;
        }
        jiffies_total = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
        jiffies_hi = (freq_avg - freq_lo) * jiffies_total;
        jiffies_hi += ((freq_hi - freq_lo) / 2);
        jiffies_hi /= (freq_hi - freq_lo);
        jiffies_lo = jiffies_total - jiffies_hi;
        dbs_info->freq_lo = freq_lo;
        dbs_info->freq_lo_jiffies = jiffies_lo;
        dbs_info->freq_hi_jiffies = jiffies_hi;
        return freq_hi;
}

static int ragingeagles_powersave_bias_setspeed(struct cpufreq_policy *policy,
                                            struct cpufreq_policy *altpolicy,
                                            int level)
{
        if (level == POWERSAVE_BIAS_MAXLEVEL) {
                
                __cpufreq_driver_target(policy,
                        (altpolicy) ? altpolicy->min : policy->min,
                        CPUFREQ_RELATION_L);
                return 1;
        } else if (level == POWERSAVE_BIAS_MINLEVEL) {
                
                __cpufreq_driver_target(policy,
                        (altpolicy) ? altpolicy->max : policy->max,
                        CPUFREQ_RELATION_H);
                return 1;
        }
        return 0;
}

static void ragingeagles_powersave_bias_init_cpu(int cpu)
{
        struct cpu_dbs_info_s *dbs_info = &per_cpu(od_cpu_dbs_info, cpu);
        dbs_info->freq_table = cpufreq_frequency_get_table(cpu);
        dbs_info->freq_lo = 0;
}

static void ragingeagles_powersave_bias_init(void)
{
        int i;
        for_each_online_cpu(i) {
                ragingeagles_powersave_bias_init_cpu(i);
        }
}

static int
dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		     void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpu_dbs_info_s *this_dbs_info = &per_cpu(cs_cpu_dbs_info,
							freq->cpu);

	struct cpufreq_policy *policy;

	if (!this_dbs_info->enable)
		return 0;

	policy = this_dbs_info->cur_policy;

	if (this_dbs_info->requested_freq > policy->max
			|| this_dbs_info->requested_freq < policy->min)
		this_dbs_info->requested_freq = freq->new;

	return 0;
}

static struct notifier_block dbs_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier
};

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_min);

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(up_threshold, up_threshold);
show_one(up_threshold_multi_core, up_threshold_multi_core);
show_one(down_differential, down_differential);
show_one(ignore_nice_load, ignore_nice);
show_one(up_threshold_any_cpu_load, up_threshold_any_cpu_load);
show_one(freq_step, freq_step);

static void update_sampling_rate(unsigned int new_rate)
{
	int cpu;

	dbs_tuners_ins.sampling_rate = new_rate
				     = max(new_rate, min_sampling_rate);

	get_online_cpus();
	for_each_online_cpu(cpu) {
		struct cpufreq_policy *policy;
		struct cpu_dbs_info_s *dbs_info;
		unsigned long next_sampling, appointed_at;

		policy = cpufreq_cpu_get(cpu);
		if (!policy)
			continue;
		dbs_info = &per_cpu(cs_cpu_dbs_info, policy->cpu);
		cpufreq_cpu_put(policy);

		mutex_lock(&dbs_info->timer_mutex);

		if (!delayed_work_pending(&dbs_info->work)) {
			mutex_unlock(&dbs_info->timer_mutex);
			continue;
		}

		next_sampling  = jiffies + usecs_to_jiffies(new_rate);
		appointed_at = dbs_info->work.timer.expires;

		if (time_before(next_sampling, appointed_at)) {

			mutex_unlock(&dbs_info->timer_mutex);
			cancel_delayed_work_sync(&dbs_info->work);
			mutex_lock(&dbs_info->timer_mutex);

			queue_delayed_work_on(dbs_info->cpu, dbs_wq,
				&dbs_info->work, usecs_to_jiffies(new_rate));

		}
		mutex_unlock(&dbs_info->timer_mutex);
	}
	put_online_cpus();
}


static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	update_sampling_rate(input);
	return count;
}

static ssize_t store_up_threshold(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.up_threshold = input;
	return count;
}

static ssize_t store_up_threshold_multi_core(struct kobject *a,
                        struct attribute *b, const char *buf, size_t count)
{
        unsigned int input;
        int ret;
        ret = sscanf(buf, "%u", &input);

        if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
                        input < MIN_FREQUENCY_UP_THRESHOLD) {
                return -EINVAL;
        }
        dbs_tuners_ins.up_threshold_multi_core = input;
        return count;
}

static ssize_t store_up_threshold_any_cpu_load(struct kobject *a,
                        struct attribute *b, const char *buf, size_t count)
{
        unsigned int input;
        int ret;
        ret = sscanf(buf, "%u", &input);

        if (ret != 1 || input > MAX_FREQUENCY_UP_THRESHOLD ||
                        input < MIN_FREQUENCY_UP_THRESHOLD) {
                return -EINVAL;
        }
        dbs_tuners_ins.up_threshold_any_cpu_load = input;
        return count;
}

static ssize_t store_down_differential(struct kobject *a, struct attribute *b,
                const char *buf, size_t count)
{
        unsigned int input;
        int ret;
        ret = sscanf(buf, "%u", &input);

        if (ret != 1 || input >= dbs_tuners_ins.up_threshold ||
                        input < MIN_FREQUENCY_DOWN_DIFFERENTIAL) {
                return -EINVAL;
        }

        dbs_tuners_ins.down_differential = input;

        return count;
}

static ssize_t store_down_differential_multi_core(struct kobject *a,
                        struct attribute *b, const char *buf, size_t count)
{
        unsigned int input;
        int ret;

        ret = sscanf(buf, "%u", &input);
        if (ret != 1)
                return -EINVAL;
        dbs_tuners_ins.down_differential_multi_core = input;
        return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) 
		return count;

	dbs_tuners_ins.ignore_nice = input;

	
	for_each_online_cpu(j) {
		struct cpu_dbs_info_s *dbs_info;
		dbs_info = &per_cpu(cs_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}

static ssize_t store_powersave_bias(struct kobject *a, struct attribute *b,
                                    const char *buf, size_t count)
{
        int input  = 0;
        int bypass = 0;
        int ret, cpu, reenable_timer, j;
        struct cpu_dbs_info_s *dbs_info;

        struct cpumask cpus_timer_done;
        cpumask_clear(&cpus_timer_done);

        ret = sscanf(buf, "%d", &input);

        if (ret != 1)
                return -EINVAL;

        if (input >= POWERSAVE_BIAS_MAXLEVEL) {
                input  = POWERSAVE_BIAS_MAXLEVEL;
                bypass = 1;
        } else if (input <= POWERSAVE_BIAS_MINLEVEL) {
                input  = POWERSAVE_BIAS_MINLEVEL;
                bypass = 1;
        }

        if (input == dbs_tuners_ins.powersave_bias) {
                
                return count;
        }

        reenable_timer = ((dbs_tuners_ins.powersave_bias ==
                                POWERSAVE_BIAS_MAXLEVEL) ||
                                (dbs_tuners_ins.powersave_bias ==
                                POWERSAVE_BIAS_MINLEVEL));

        dbs_tuners_ins.powersave_bias = input;

        get_online_cpus();
        mutex_lock(&dbs_mutex);

        if (!bypass) {
                if (reenable_timer) {
                        
                        for_each_online_cpu(cpu) {
                                if (lock_policy_rwsem_write(cpu) < 0)
                                        continue;

                                dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

                                if (!dbs_info->cur_policy) {
                                        pr_err("Dbs policy is NULL\n");
                                        goto skip_this_cpu;
                                }

                                for_each_cpu(j, &cpus_timer_done) {
                                        if (cpumask_test_cpu(j, dbs_info->
                                                        cur_policy->cpus))
                                                goto skip_this_cpu;
                                }

                                cpumask_set_cpu(cpu, &cpus_timer_done);
                                if (dbs_info->cur_policy) {
                                        dbs_timer_exit(dbs_info);
                                        
                                        mutex_lock(&dbs_info->timer_mutex);
                                        dbs_timer_init(dbs_info);
#ifndef CONFIG_ARCH_MSM_CORTEXMP
                                        mutex_unlock(&dbs_info->timer_mutex);
                                        atomic_set(&dbs_info->sync_enabled, 1);
#endif
                                }
skip_this_cpu:
                                unlock_policy_rwsem_write(cpu);
                        }
                }
                ragingeagles_powersave_bias_init();
        } else {
                for_each_online_cpu(cpu) {
                        if (lock_policy_rwsem_write(cpu) < 0)
                                continue;

                        dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

                        if (!dbs_info->cur_policy) {
                                pr_err("Dbs policy is NULL\n");
                                goto skip_this_cpu_bypass;
                        }

                        for_each_cpu(j, &cpus_timer_done) {
                                if (cpumask_test_cpu(j, dbs_info->
                                                        cur_policy->cpus))
                                        goto skip_this_cpu_bypass;
                        }

                        cpumask_set_cpu(cpu, &cpus_timer_done);

                        if (dbs_info->cur_policy) {
                                
                                dbs_timer_exit(dbs_info);
#ifndef CONFIG_ARCH_MSM_CORTEXMP
                                atomic_set(&dbs_info->sync_enabled, 0);
#endif

                                mutex_lock(&dbs_info->timer_mutex);
                                ragingeagles_powersave_bias_setspeed(
                                        dbs_info->cur_policy,
                                        NULL,
                                        input);
                                mutex_unlock(&dbs_info->timer_mutex);

                        }
skip_this_cpu_bypass:
                        unlock_policy_rwsem_write(cpu);
                }
        }

        mutex_unlock(&dbs_mutex);
        put_online_cpus();

        return count;
}

static ssize_t store_freq_step(struct kobject *a, struct attribute *b,
			       const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	if (input > 100)
		input = 100;

	/* no need to test here if freq_step is zero as the user might actually
	 * want this, they would be crazy though :) */
	dbs_tuners_ins.freq_step = input;
	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(up_threshold);
define_one_global_rw(down_differential);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(powersave_bias);
define_one_global_rw(up_threshold_multi_core);
define_one_global_rw(up_threshold_any_cpu_load);
define_one_global_rw(freq_step);

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&up_threshold.attr,
        &down_differential.attr,
        &powersave_bias.attr,
        &up_threshold_multi_core.attr,
        &up_threshold_any_cpu_load.attr,
        &freq_step.attr,
	&ignore_nice_load.attr,
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "ragingeagles",
};

static void dbs_check_cpu(struct cpu_dbs_info_s *this_dbs_info)
{
	unsigned int load = 0;
	unsigned int max_load = 0;
	unsigned int freq_next;
	struct cpufreq_policy *policy;
	unsigned int j;

	policy = this_dbs_info->cur_policy;

	
	for_each_cpu(j, policy->cpus) {
		struct cpu_dbs_info_s *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;

		j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);

		wall_time = (unsigned int)
			(cur_wall_time - j_dbs_info->prev_cpu_wall);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int)
			(cur_idle_time - j_dbs_info->prev_cpu_idle);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		if (dbs_tuners_ins.ignore_nice) {
			u64 cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
					 j_dbs_info->prev_cpu_nice;
			cur_nice_jiffies = (unsigned long)
					cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		load = 100 * (wall_time - idle_time) / wall_time;

		if (load > max_load)
			max_load = load;
	}

	if (dbs_tuners_ins.freq_step == 0)
		return;

	if (dbs_tuners_ins.gboost) {
		
		if (g_count < 100) {
			++g_count;
			++g_count;
		} else if (g_count > 2) {
			--g_count;
		}

		if (g_count > 10) {
			dbs_tuners_ins.shortcut = 1;
		} else {
			dbs_tuners_ins.shortcut = 0;
		}
	}


	if (max_load < dbs_tuners_ins.up_threshold ) {
	/*if load is low then bias to lower speeds*/

		freq_next = (dbs_tuners_ins.freq_step * policy->max) / 100;

		this_dbs_info->requested_freq -= freq_next;
		if (this_dbs_info->requested_freq < policy->min)
			this_dbs_info->requested_freq = policy->min;

		if (freq_next < policy->min)
			freq_next = policy->min;
		__cpufreq_driver_target(policy, this_dbs_info->requested_freq,
				CPUFREQ_RELATION_H);
		return;

	}

	if (max_load > dbs_tuners_ins.up_threshold ){
		this_dbs_info->down_skip = 0;
		/* Calculate the next frequency proportional to load */
		freq_next = load * policy->max / 100;

		if (freq_next > policy->max)
			freq_next = policy->max;

		__cpufreq_driver_target(policy, freq_next,
				CPUFREQ_RELATION_L);
			return;
	} else if (dbs_tuners_ins.shortcut){ 
		freq_next = policy->max;
	
		__cpufreq_driver_target(policy, freq_next,
				CPUFREQ_RELATION_L);
		return;
	}

}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpu_dbs_info_s *dbs_info =
		container_of(work, struct cpu_dbs_info_s, work.work);
	unsigned int cpu = dbs_info->cpu;

	
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);

	delay -= jiffies % delay;

	mutex_lock(&dbs_info->timer_mutex);

	dbs_check_cpu(dbs_info);

	queue_delayed_work_on(cpu, dbs_wq, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpu_dbs_info_s *dbs_info)
{
	
	int delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate);
	delay -= jiffies % delay;

	dbs_info->enable = 1;
	INIT_DELAYED_WORK_DEFERRABLE(&dbs_info->work, do_dbs_timer);
	queue_delayed_work_on(dbs_info->cpu, dbs_wq, &dbs_info->work, delay);
}

static inline void dbs_timer_exit(struct cpu_dbs_info_s *dbs_info)
{
	dbs_info->enable = 0;
	cancel_delayed_work_sync(&dbs_info->work);
}

static void dbs_input_event(struct input_handle *handle, unsigned int type,
                unsigned int code, int value)
{
        int i;
        struct cpu_dbs_info_s *dbs_info;
        unsigned long flags;
        int input_event_min_freq = 0;

        if ((dbs_tuners_ins.powersave_bias == POWERSAVE_BIAS_MAXLEVEL) ||
                (dbs_tuners_ins.powersave_bias == POWERSAVE_BIAS_MINLEVEL)) {
                
                return;
        }

        if (type == EV_SYN && code == SYN_REPORT) {
                
                dbs_tuners_ins.powersave_bias = 0;
        }
        else if (type == EV_ABS && code == ABS_MT_TRACKING_ID) {
                
                if (value != -1) {
                        input_event_counter++;
                        input_event_min_freq = input_event_min_freq_array[num_online_cpus() - 1];
                        
                        switch_turbo_mode(0);
                }
                
                else {
                        if (likely(input_event_counter > 0))
                                input_event_counter--;
                        else
                                pr_warning("dbs_input_event: Touch isn't paired!\n");

                        input_event_min_freq = 0;
                        
                        switch_turbo_mode(DBS_SWITCH_MODE_TIMEOUT);
                }
        }
        else if (type == EV_KEY && value == 1 &&
                        (code == KEY_POWER || code == KEY_VOLUMEUP || code == KEY_VOLUMEDOWN))
        {
                input_event_min_freq = input_event_min_freq_array[num_online_cpus() - 1];
                
                switch_turbo_mode(DBS_SWITCH_MODE_TIMEOUT);
        }

        if (input_event_min_freq > 0) {
                
                spin_lock_irqsave(&input_boost_lock, flags);
                input_event_boost = true;
                input_event_boost_expired = jiffies + usecs_to_jiffies(dbs_tuners_ins.sampling_rate * 4);
                spin_unlock_irqrestore(&input_boost_lock, flags);

                for_each_online_cpu(i)
                {
#ifdef CONFIG_ARCH_MSM_CORTEXMP
                        if (i != CPU0)
                                break;
#endif
                        dbs_info = &per_cpu(od_cpu_dbs_info, i);
                         if (dbs_info->cur_policy &&             
                                dbs_info->cur_policy->cur < input_event_min_freq) {
                                dbs_info->input_event_freq = input_event_min_freq;
                                wake_up_process(per_cpu(up_task, i));
                        }
                }
        }
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
                                   unsigned int event)
{
        unsigned int cpu = policy->cpu;
        struct cpu_dbs_info_s *this_dbs_info;
        unsigned int j;
        int rc;

        this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

        switch (event) {
        case CPUFREQ_GOV_START:
                if ((!cpu_online(cpu)) || (!policy->cur))
                        return -EINVAL;

                mutex_lock(&dbs_mutex);

                dbs_enable++;
                for_each_cpu(j, policy->cpus) {
                        struct cpu_dbs_info_s *j_dbs_info;
                        j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
                        j_dbs_info->cur_policy = policy;

                        j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
                                                &j_dbs_info->prev_cpu_wall);
                        if (dbs_tuners_ins.ignore_nice)
                                j_dbs_info->prev_cpu_nice =
                                                kcpustat_cpu(j).cpustat[CPUTIME_NICE];
#ifndef CONFIG_ARCH_MSM_CORTEXMP
                        set_cpus_allowed(j_dbs_info->sync_thread,
                                         *cpumask_of(j));
                        if (!dbs_tuners_ins.powersave_bias)
                                atomic_set(&j_dbs_info->sync_enabled, 1);
#endif
                }
                this_dbs_info->cpu = cpu;
                this_dbs_info->rate_mult = 1;
                ragingeagles_powersave_bias_init_cpu(cpu);
                if (dbs_enable == 1) {
                        unsigned int latency;

                        rc = sysfs_create_group(cpufreq_global_kobject,
                                                &dbs_attr_group);
                        if (rc) {
                                mutex_unlock(&dbs_mutex);
                                return rc;
                        }

                        
                        latency = policy->cpuinfo.transition_latency / 1000;
                        if (latency == 0)
                                latency = 1;
                        
                        min_sampling_rate = max(min_sampling_rate,
                                        MIN_LATENCY_MULTIPLIER * latency);
                        dbs_tuners_ins.sampling_rate =
                                max(min_sampling_rate,
                                    latency * LATENCY_MULTIPLIER);

                        if (dbs_tuners_ins.sampling_rate < DEF_SAMPLING_RATE)
                                 dbs_tuners_ins.sampling_rate = DEF_SAMPLING_RATE;

                        dbs_tuners_ins.io_is_busy = should_io_be_busy();

                        if (dbs_tuners_ins.optimal_freq == 0)
                                dbs_tuners_ins.optimal_freq = policy->min;

                        if (dbs_tuners_ins.sync_freq == 0)
                                dbs_tuners_ins.sync_freq = policy->min;

                        dbs_init_freq_map_table(policy);

#ifndef CONFIG_ARCH_MSM_CORTEXMP
                        atomic_notifier_chain_register(&migration_notifier_head,
                                        &dbs_migration_nb);
#endif
                }
                if (!cpu)
                        rc = input_register_handler(&dbs_input_handler);
                mutex_unlock(&dbs_mutex);

                if (!ragingeagles_powersave_bias_setspeed(
                                        this_dbs_info->cur_policy,
                                        NULL,
                                        dbs_tuners_ins.powersave_bias))
                        dbs_timer_init(this_dbs_info);
                break;

        case CPUFREQ_GOV_STOP:
                dbs_timer_exit(this_dbs_info);
                this_dbs_info->prev_load = 0;

                mutex_lock(&dbs_mutex);
                dbs_enable--;

#ifndef CONFIG_ARCH_MSM_CORTEXMP
                for_each_cpu(j, policy->cpus) {
                        struct cpu_dbs_info_s *j_dbs_info;
                        j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
                        atomic_set(&j_dbs_info->sync_enabled, 0);
                }
#endif

                this_dbs_info->cur_policy = NULL;
                if (!cpu)
                        input_unregister_handler(&dbs_input_handler);
                if (!dbs_enable) {
                        dbs_deinit_freq_map_table();

                        sysfs_remove_group(cpufreq_global_kobject,
                                           &dbs_attr_group);

#ifndef CONFIG_ARCH_MSM_CORTEXMP
                        atomic_notifier_chain_unregister(
                                &migration_notifier_head,
                                &dbs_migration_nb);
#endif
                }

                mutex_unlock(&dbs_mutex);
                trace_cpufreq_interactive_target (cpu, 0, 0, 0, 0);
                break;

        case CPUFREQ_GOV_LIMITS:
                mutex_lock(&this_dbs_info->timer_mutex);
                if(this_dbs_info->cur_policy) {
                        if (policy->max < this_dbs_info->cur_policy->cur)
                                __cpufreq_driver_target(this_dbs_info->cur_policy,
                                        policy->max, CPUFREQ_RELATION_H);
                        else if (policy->min > this_dbs_info->cur_policy->cur)
                                __cpufreq_driver_target(this_dbs_info->cur_policy,
                                        policy->min, CPUFREQ_RELATION_L);
                        else if (dbs_tuners_ins.powersave_bias != 0)
                                ragingeagles_powersave_bias_setspeed(
                                        this_dbs_info->cur_policy,
                                        policy,
                                        dbs_tuners_ins.powersave_bias);
                }
                mutex_unlock(&this_dbs_info->timer_mutex);
                break;
        }
        return 0;
}

static int cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpu_dbs_info_s *this_dbs_info;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(cs_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&dbs_mutex);

		for_each_cpu(j, policy->cpus) {
			struct cpu_dbs_info_s *j_dbs_info;
			j_dbs_info = &per_cpu(cs_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
						&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice)
				j_dbs_info->prev_cpu_nice =
						kcpustat_cpu(j).cpustat[CPUTIME_NICE];
		}
		this_dbs_info->down_skip = 0;
		this_dbs_info->requested_freq = policy->cur;

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_enable++;
		if (dbs_enable == 1) {
			unsigned int latency;
			
			latency = policy->cpuinfo.transition_latency / 1000;
			if (latency == 0)
				latency = 1;

			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			min_sampling_rate = max(min_sampling_rate,
					MIN_LATENCY_MULTIPLIER * latency);
			dbs_tuners_ins.sampling_rate =
				max(min_sampling_rate,
				    latency * LATENCY_MULTIPLIER);

			if (dbs_tuners_ins.sampling_rate < DEF_SAMPLING_RATE)
				 dbs_tuners_ins.sampling_rate = DEF_SAMPLING_RATE;

			cpufreq_register_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}
		mutex_unlock(&dbs_mutex);

		dbs_timer_init(this_dbs_info);

		break;

	case CPUFREQ_GOV_STOP:
		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		dbs_enable--;
		mutex_destroy(&this_dbs_info->timer_mutex);

		if (dbs_enable == 0)
			cpufreq_unregister_notifier(
					&dbs_cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);

		mutex_unlock(&dbs_mutex);
		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);
		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(
					this_dbs_info->cur_policy,
					policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_dbs_info->timer_mutex);

		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_RAGINGEAGLES
static
#endif
struct cpufreq_governor cpufreq_gov_ragingeagles = {
	.name			= "ragingeagles",
	.governor		= cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	dbs_wq = alloc_workqueue("ragingeagles_dbs_wq", WQ_HIGHPRI, 0);
	if (!dbs_wq) {
		printk(KERN_ERR "Failed to create ragingeagles_dbs_wq workqueue\n");
		return -EFAULT;
	}

	return cpufreq_register_governor(&cpufreq_gov_ragingeagles);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_ragingeagles);
	destroy_workqueue(dbs_wq);
}


MODULE_AUTHOR("Alexander Clouter <alex@digriz.org.uk>");
MODULE_DESCRIPTION("'cpufreq_ragingeagles' - A dynamic cpufreq governor for Venturization of your mind. "
		"Low Latency Frequency Transition capable processors "
		"optimised for use in a battery environment. ¯\_(ツ)_/¯");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_RAGINGEAGLES
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
