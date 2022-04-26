/*
 *  drivers/cpufreq/cpufreq_stats.c
 *
 *  Copyright (C) 2003-2004 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *  (C) 2004 Zou Nan hai <nanhai.zou@intel.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/slab.h>

static DEFINE_SPINLOCK(cpufreq_stats_lock);
#ifdef CONFIG_OPLUS_FEATURE_MIDAS
static DEFINE_SPINLOCK(idle_lock);
#endif

struct cpufreq_stats {
	unsigned int total_trans;
	unsigned long long last_time;
	unsigned int max_state;
	unsigned int state_num;
	unsigned int last_index;
	u64 *time_in_state;
	unsigned int *freq_table;
	unsigned int *trans_table;
#ifdef CONFIG_OPLUS_FEATURE_MIDAS
	/* stats for idle time of per freq */
	unsigned int nr_cpu;
	unsigned int *in_idle;
	unsigned long long *last_idle_time;
	u64 **idle_in_state;
#endif
};

static void cpufreq_stats_update(struct cpufreq_stats *stats)
{
	unsigned long long cur_time = get_jiffies_64();
	unsigned long flags;
#ifdef CONFIG_OPLUS_FEATURE_MIDAS
	unsigned int i;
#endif

	spin_lock_irqsave(&cpufreq_stats_lock, flags);
	stats->time_in_state[stats->last_index] += cur_time - stats->last_time;
	stats->last_time = cur_time;
#ifdef CONFIG_OPLUS_FEATURE_MIDAS
	for (i = 0; i < stats->nr_cpu; i++) {
		if (stats->in_idle[i]) {
			if (stats->last_idle_time[i])
				stats->idle_in_state[i][stats->last_index] += cur_time - stats->last_idle_time[i];

			stats->last_idle_time[i] = cur_time;
		}
	}
#endif
	spin_unlock_irqrestore(&cpufreq_stats_lock, flags);
}

#ifdef CONFIG_OPLUS_FEATURE_MIDAS
void cpufreq_stats_idle_hook(unsigned int cpu, unsigned int in_idle) {
	struct cpufreq_policy *policy;
	struct cpufreq_stats *stats;
	unsigned long flags;
	int first_cpu;

	policy = cpufreq_cpu_get(cpu);
	if (IS_ERR_OR_NULL(policy)) {
		pr_err("%s: policy %d is invalid", __func__, cpu);
		return;
	}

	stats = policy->stats;
	if (IS_ERR_OR_NULL(stats)) {
		pr_err("%s: no stats found\n", __func__);
		cpufreq_cpu_put(policy);
		return;
	}

	first_cpu = cpumask_first(policy->related_cpus);
	cpu -= first_cpu;

	spin_lock_irqsave(&idle_lock, flags);

	stats->in_idle[cpu] = in_idle;
	if (!in_idle)
		stats->last_idle_time[cpu] = 0;

	spin_unlock_irqrestore(&idle_lock, flags);

	cpufreq_stats_update(stats);
	cpufreq_cpu_put(policy);
}
EXPORT_SYMBOL_GPL(cpufreq_stats_idle_hook);
#endif

static void cpufreq_stats_clear_table(struct cpufreq_stats *stats)
{
	unsigned int count = stats->max_state;

	memset(stats->time_in_state, 0, count * sizeof(u64));
	memset(stats->trans_table, 0, count * count * sizeof(int));
	stats->last_time = get_jiffies_64();
	stats->total_trans = 0;
}

static ssize_t show_total_trans(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%d\n", policy->stats->total_trans);
}

static ssize_t show_time_in_state(struct cpufreq_policy *policy, char *buf)
{
	struct cpufreq_stats *stats = policy->stats;
	ssize_t len = 0;
#ifdef CONFIG_OPLUS_FEATURE_MIDAS
	int i, j;
#else
	int i;
#endif

	cpufreq_stats_update(stats);
#ifdef CONFIG_OPLUS_FEATURE_MIDAS
	for (i = 0; i < stats->state_num; i++) {
		len += sprintf(buf + len, "%u %llu ", stats->freq_table[i],
			(unsigned long long)
			jiffies_64_to_clock_t(stats->time_in_state[i]));
		for (j = 0; j < stats->nr_cpu; j++) {
			if (j < stats->nr_cpu - 1)
				len += sprintf(buf + len, "%llu ", (unsigned long long)
					jiffies_64_to_clock_t(stats->idle_in_state[j][i]));
			else
				len += sprintf(buf + len, "%llu\n", (unsigned long long)
					jiffies_64_to_clock_t(stats->idle_in_state[j][i]));
		}
	}
#else
	for (i = 0; i < stats->state_num; i++) {
		len += sprintf(buf + len, "%u %llu\n", stats->freq_table[i],
			(unsigned long long)
			jiffies_64_to_clock_t(stats->time_in_state[i]));
	}
#endif
	return len;
}

static ssize_t store_reset(struct cpufreq_policy *policy, const char *buf,
			   size_t count)
{
	/* We don't care what is written to the attribute. */
	cpufreq_stats_clear_table(policy->stats);
	return count;
}

static ssize_t show_trans_table(struct cpufreq_policy *policy, char *buf)
{
	struct cpufreq_stats *stats = policy->stats;
	ssize_t len = 0;
	int i, j;

	len += snprintf(buf + len, PAGE_SIZE - len, "   From  :    To\n");
	len += snprintf(buf + len, PAGE_SIZE - len, "         : ");
	for (i = 0; i < stats->state_num; i++) {
		if (len >= PAGE_SIZE)
			break;
		len += snprintf(buf + len, PAGE_SIZE - len, "%9u ",
				stats->freq_table[i]);
	}
	if (len >= PAGE_SIZE)
		return PAGE_SIZE;

	len += snprintf(buf + len, PAGE_SIZE - len, "\n");

	for (i = 0; i < stats->state_num; i++) {
		if (len >= PAGE_SIZE)
			break;

		len += snprintf(buf + len, PAGE_SIZE - len, "%9u: ",
				stats->freq_table[i]);

		for (j = 0; j < stats->state_num; j++) {
			if (len >= PAGE_SIZE)
				break;
			len += snprintf(buf + len, PAGE_SIZE - len, "%9u ",
					stats->trans_table[i*stats->max_state+j]);
		}
		if (len >= PAGE_SIZE)
			break;
		len += snprintf(buf + len, PAGE_SIZE - len, "\n");
	}

	if (len >= PAGE_SIZE) {
		pr_warn_once("cpufreq transition table exceeds PAGE_SIZE. Disabling\n");
		return -EFBIG;
	}
	return len;
}
cpufreq_freq_attr_ro(trans_table);

cpufreq_freq_attr_ro(total_trans);
cpufreq_freq_attr_ro(time_in_state);
cpufreq_freq_attr_wo(reset);

static struct attribute *default_attrs[] = {
	&total_trans.attr,
	&time_in_state.attr,
	&reset.attr,
	&trans_table.attr,
	NULL
};
static const struct attribute_group stats_attr_group = {
	.attrs = default_attrs,
	.name = "stats"
};

static int freq_table_get_index(struct cpufreq_stats *stats, unsigned int freq)
{
	int index;
	for (index = 0; index < stats->max_state; index++)
		if (stats->freq_table[index] == freq)
			return index;
	return -1;
}

void cpufreq_stats_free_table(struct cpufreq_policy *policy)
{
	struct cpufreq_stats *stats = policy->stats;

	/* Already freed */
	if (!stats)
		return;

	pr_debug("%s: Free stats table\n", __func__);

	sysfs_remove_group(&policy->kobj, &stats_attr_group);
	kfree(stats->time_in_state);
	kfree(stats);
	policy->stats = NULL;
}

void cpufreq_stats_create_table(struct cpufreq_policy *policy)
{
	unsigned int i = 0, count = 0, ret = -ENOMEM;
	struct cpufreq_stats *stats;
	unsigned int alloc_size;
	struct cpufreq_frequency_table *pos;
#ifdef CONFIG_OPLUS_FEATURE_MIDAS
	unsigned int nr_cpu;
#endif

	count = cpufreq_table_count_valid_entries(policy);
	if (!count)
		return;

	/* stats already initialized */
	if (policy->stats)
		return;

	stats = kzalloc(sizeof(*stats), GFP_KERNEL);
	if (!stats)
		return;

	alloc_size = count * sizeof(int) + count * sizeof(u64);

	alloc_size += count * count * sizeof(int);

	/* Allocate memory for time_in_state/freq_table/trans_table in one go */
	stats->time_in_state = kzalloc(alloc_size, GFP_KERNEL);
	if (!stats->time_in_state)
		goto free_stat;

	stats->freq_table = (unsigned int *)(stats->time_in_state + count);

	stats->trans_table = stats->freq_table + count;

	stats->max_state = count;

	/* Find valid-unique entries */
	cpufreq_for_each_valid_entry(pos, policy->freq_table)
		if (freq_table_get_index(stats, pos->frequency) == -1)
			stats->freq_table[i++] = pos->frequency;

	stats->state_num = i;
	stats->last_time = get_jiffies_64();
	stats->last_index = freq_table_get_index(stats, policy->cur);

#ifdef CONFIG_OPLUS_FEATURE_MIDAS
	/* Allocate memory for idle_in_state/last_idle_time/in_idle */
	nr_cpu = cpumask_weight(policy->related_cpus);
	stats->nr_cpu = nr_cpu;
	stats->in_idle = kzalloc(nr_cpu * sizeof(unsigned int), GFP_KERNEL);
	if (!stats->in_idle)
		goto free_time_in_state;

	stats->last_idle_time = kzalloc(nr_cpu * sizeof(unsigned long long),
				GFP_KERNEL);
	if (!stats->last_idle_time)
		goto free_in_idle;

	stats->idle_in_state = kzalloc(nr_cpu * sizeof(u64 *), GFP_KERNEL);
	if (!stats->idle_in_state)
		goto free_last_idle_time;

	for (i = 0; i < nr_cpu; i++) {
		stats->idle_in_state[i] = kzalloc(alloc_size, GFP_KERNEL);
		if (!stats->idle_in_state[i])
			goto free_idle_in_state;
	}
#endif
	policy->stats = stats;
	ret = sysfs_create_group(&policy->kobj, &stats_attr_group);
	if (!ret)
		return;

	/* We failed, release resources */
	policy->stats = NULL;

#ifdef CONFIG_OPLUS_FEATURE_MIDAS
free_idle_in_state:
	for (i--; i >= 0; i--)
		kfree(stats->idle_in_state[i]);

	kfree(stats->idle_in_state);
free_last_idle_time:
	kfree(stats->last_idle_time);
free_in_idle:
	kfree(stats->in_idle);
free_time_in_state:
	kfree(stats->time_in_state);
#endif
free_stat:
	kfree(stats);
}

void cpufreq_stats_record_transition(struct cpufreq_policy *policy,
				     unsigned int new_freq)
{
	struct cpufreq_stats *stats = policy->stats;
	int old_index, new_index;

	if (!stats) {
		pr_debug("%s: No stats found\n", __func__);
		return;
	}

	old_index = stats->last_index;
	new_index = freq_table_get_index(stats, new_freq);

	/* We can't do stats->time_in_state[-1]= .. */
	if (old_index == -1 || new_index == -1 || old_index == new_index)
		return;

	cpufreq_stats_update(stats);

	stats->last_index = new_index;
	stats->trans_table[old_index * stats->max_state + new_index]++;
	stats->total_trans++;
}
