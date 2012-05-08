#define _FILE_OFFSET_BITS 64

#include <linux/kernel.h>

#include <byteswap.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/mman.h>

#include "evlist.h"
#include "evsel.h"
#include "session.h"
#include "sort.h"
#include "util.h"

static int perf_session__open(struct perf_session *self, bool force)
{
	struct stat input_stat;

	if (!strcmp(self->filename, "-")) {
		self->fd_pipe = true;
		self->fd = STDIN_FILENO;

		if (perf_session__read_header(self, self->fd) < 0)
			pr_err("incompatible file format");

		return 0;
	}

	self->fd = open(self->filename, O_RDONLY);
	if (self->fd < 0) {
		int err = errno;

		pr_err("failed to open %s: %s", self->filename, strerror(err));
		if (err == ENOENT && !strcmp(self->filename, "perf.data"))
			pr_err("  (try 'perf record' first)");
		pr_err("\n");
		return -errno;
	}

	if (fstat(self->fd, &input_stat) < 0)
		goto out_close;

	if (!force && input_stat.st_uid && (input_stat.st_uid != geteuid())) {
		pr_err("file %s not owned by current user or root\n",
		       self->filename);
		goto out_close;
	}

	if (!input_stat.st_size) {
		pr_info("zero-sized file (%s), nothing to do!\n",
			self->filename);
		goto out_close;
	}

	if (perf_session__read_header(self, self->fd) < 0) {
		pr_err("incompatible file format");
		goto out_close;
	}

	if (!perf_evlist__valid_sample_type(self->evlist)) {
		pr_err("non matching sample_type");
		goto out_close;
	}

	if (!perf_evlist__valid_sample_id_all(self->evlist)) {
		pr_err("non matching sample_id_all");
		goto out_close;
	}

	self->size = input_stat.st_size;
	return 0;

out_close:
	close(self->fd);
	self->fd = -1;
	return -1;
}

static void perf_session__id_header_size(struct perf_session *session)
{
       struct perf_sample *data;
       u64 sample_type = session->sample_type;
       u16 size = 0;

	if (!session->sample_id_all)
		goto out;

       if (sample_type & PERF_SAMPLE_TID)
               size += sizeof(data->tid) * 2;

       if (sample_type & PERF_SAMPLE_TIME)
               size += sizeof(data->time);

       if (sample_type & PERF_SAMPLE_ID)
               size += sizeof(data->id);

       if (sample_type & PERF_SAMPLE_STREAM_ID)
               size += sizeof(data->stream_id);

       if (sample_type & PERF_SAMPLE_CPU)
               size += sizeof(data->cpu) * 2;
out:
       session->id_hdr_size = size;
}

void perf_session__update_sample_type(struct perf_session *self)
{
	self->sample_type = perf_evlist__sample_type(self->evlist);
	self->sample_size = __perf_evsel__sample_size(self->sample_type);
	self->sample_id_all = perf_evlist__sample_id_all(self->evlist);
	perf_session__id_header_size(self);
}

int perf_session__create_kernel_maps(struct perf_session *self)
{
	int ret = machine__create_kernel_maps(&self->host_machine);

	if (ret >= 0)
		ret = machines__create_guest_kernel_maps(&self->machines);
	return ret;
}

static void perf_session__destroy_kernel_maps(struct perf_session *self)
{
	machine__destroy_kernel_maps(&self->host_machine);
	machines__destroy_guest_kernel_maps(&self->machines);
}

struct perf_session *perf_session__new(const char *filename, int mode,
				       bool force, bool repipe,
				       struct perf_event_ops *ops)
{
	size_t len = filename ? strlen(filename) + 1 : 0;
	struct perf_session *self = zalloc(sizeof(*self) + len);

	if (self == NULL)
		goto out;

	memcpy(self->filename, filename, len);
	self->threads = RB_ROOT;
	INIT_LIST_HEAD(&self->dead_threads);
	self->last_match = NULL;
	/*
	 * On 64bit we can mmap the data file in one go. No need for tiny mmap
	 * slices. On 32bit we use 32MB.
	 */
#if BITS_PER_LONG == 64
	self->mmap_window = ULLONG_MAX;
#else
	self->mmap_window = 32 * 1024 * 1024ULL;
#endif
	self->machines = RB_ROOT;
	self->repipe = repipe;
	INIT_LIST_HEAD(&self->ordered_samples.samples);
	INIT_LIST_HEAD(&self->ordered_samples.sample_cache);
	INIT_LIST_HEAD(&self->ordered_samples.to_free);
	machine__init(&self->host_machine, "", HOST_KERNEL_ID);

	if (mode == O_RDONLY) {
		if (perf_session__open(self, force) < 0)
			goto out_delete;
		perf_session__update_sample_type(self);
	} else if (mode == O_WRONLY) {
		/*
		 * In O_RDONLY mode this will be performed when reading the
		 * kernel MMAP event, in perf_event__process_mmap().
		 */
		if (perf_session__create_kernel_maps(self) < 0)
			goto out_delete;
	}

	if (ops && ops->ordering_requires_timestamps &&
	    ops->ordered_samples && !self->sample_id_all) {
		dump_printf("WARNING: No sample_id_all support, falling back to unordered processing\n");
		ops->ordered_samples = false;
	}

out:
	return self;
out_delete:
	perf_session__delete(self);
	return NULL;
}

static void perf_session__delete_dead_threads(struct perf_session *self)
{
	struct thread *n, *t;

	list_for_each_entry_safe(t, n, &self->dead_threads, node) {
		list_del(&t->node);
		thread__delete(t);
	}
}

static void perf_session__delete_threads(struct perf_session *self)
{
	struct rb_node *nd = rb_first(&self->threads);

	while (nd) {
		struct thread *t = rb_entry(nd, struct thread, rb_node);

		rb_erase(&t->rb_node, &self->threads);
		nd = rb_next(nd);
		thread__delete(t);
	}
}

void perf_session__delete(struct perf_session *self)
{
	perf_session__destroy_kernel_maps(self);
	perf_session__delete_dead_threads(self);
	perf_session__delete_threads(self);
	machine__exit(&self->host_machine);
	close(self->fd);
	free(self);
}

void perf_session__remove_thread(struct perf_session *self, struct thread *th)
{
	self->last_match = NULL;
	rb_erase(&th->rb_node, &self->threads);
	/*
	 * We may have references to this thread, for instance in some hist_entry
	 * instances, so just move them to a separate list.
	 */
	list_add_tail(&th->node, &self->dead_threads);
}

static bool symbol__match_parent_regex(struct symbol *sym)
{
	if (sym->name && !regexec(&parent_regex, sym->name, 0, NULL, 0))
		return 1;

	return 0;
}

int perf_session__resolve_callchain(struct perf_session *self,
				    struct thread *thread,
				    struct ip_callchain *chain,
				    struct symbol **parent)
{
	u8 cpumode = PERF_RECORD_MISC_USER;
	unsigned int i;
	int err;

	callchain_cursor_reset(&self->callchain_cursor);

	for (i = 0; i < chain->nr; i++) {
		u64 ip = chain->ips[i];
		struct addr_location al;

		if (ip >= PERF_CONTEXT_MAX) {
			switch (ip) {
			case PERF_CONTEXT_HV:
				cpumode = PERF_RECORD_MISC_HYPERVISOR;	break;
			case PERF_CONTEXT_KERNEL:
				cpumode = PERF_RECORD_MISC_KERNEL;	break;
			case PERF_CONTEXT_USER:
				cpumode = PERF_RECORD_MISC_USER;	break;
			default:
				break;
			}
			continue;
		}

		al.filtered = false;
		thread__find_addr_location(thread, self, cpumode,
				MAP__FUNCTION, thread->pid, ip, &al, NULL);
		if (al.sym != NULL) {
			if (sort__has_parent && !*parent &&
			    symbol__match_parent_regex(al.sym))
				*parent = al.sym;
			if (!symbol_conf.use_callchain)
				break;
		}

		err = callchain_cursor_append(&self->callchain_cursor,
					      ip, al.map, al.sym);
		if (err)
			return err;
	}

	return 0;
}

static int process_event_synth_stub(union perf_event *event __used,
				    struct perf_session *session __used)
{
	dump_printf(": unhandled!\n");
	return 0;
}

static int process_event_sample_stub(union perf_event *event __used,
				     struct perf_sample *sample __used,
				     struct perf_evsel *evsel __used,
				     struct perf_session *session __used)
{
	dump_printf(": unhandled!\n");
	return 0;
}

static int process_event_stub(union perf_event *event __used,
			      struct perf_sample *sample __used,
			      struct perf_session *session __used)
{
	dump_printf(": unhandled!\n");
	return 0;
}

static int process_finished_round_stub(union perf_event *event __used,
				       struct perf_session *session __used,
				       struct perf_event_ops *ops __used)
{
	dump_printf(": unhandled!\n");
	return 0;
}

static int process_finished_round(union perf_event *event,
				  struct perf_session *session,
				  struct perf_event_ops *ops);

static void perf_event_ops__fill_defaults(struct perf_event_ops *handler)
{
	if (handler->sample == NULL)
		handler->sample = process_event_sample_stub;
	if (handler->mmap == NULL)
		handler->mmap = process_event_stub;
	if (handler->comm == NULL)
		handler->comm = process_event_stub;
	if (handler->fork == NULL)
		handler->fork = process_event_stub;
	if (handler->exit == NULL)
		handler->exit = process_event_stub;
	if (handler->lost == NULL)
		handler->lost = perf_event__process_lost;
	if (handler->read == NULL)
		handler->read = process_event_stub;
	if (handler->throttle == NULL)
		handler->throttle = process_event_stub;
	if (handler->unthrottle == NULL)
		handler->unthrottle = process_event_stub;
	if (handler->attr == NULL)
		handler->attr = process_event_synth_stub;
	if (handler->event_type == NULL)
		handler->event_type = process_event_synth_stub;
	if (handler->tracing_data == NULL)
		handler->tracing_data = process_event_synth_stub;
	if (handler->build_id == NULL)
		handler->build_id = process_event_synth_stub;
	if (handler->finished_round == NULL) {
		if (handler->ordered_samples)
			handler->finished_round = process_finished_round;
		else
			handler->finished_round = process_finished_round_stub;
	}
}

void mem_bswap_64(void *src, int byte_size)
{
	u64 *m = src;

	while (byte_size > 0) {
		*m = bswap_64(*m);
		byte_size -= sizeof(u64);
		++m;
	}
}

static void perf_event__all64_swap(union perf_event *event)
{
	struct perf_event_header *hdr = &event->header;
	mem_bswap_64(hdr + 1, event->header.size - sizeof(*hdr));
}

static void perf_event__comm_swap(union perf_event *event)
{
	event->comm.pid = bswap_32(event->comm.pid);
	event->comm.tid = bswap_32(event->comm.tid);
}

static void perf_event__mmap_swap(union perf_event *event)
{
	event->mmap.pid	  = bswap_32(event->mmap.pid);
	event->mmap.tid	  = bswap_32(event->mmap.tid);
	event->mmap.start = bswap_64(event->mmap.start);
	event->mmap.len	  = bswap_64(event->mmap.len);
	event->mmap.pgoff = bswap_64(event->mmap.pgoff);
}

static void perf_event__task_swap(union perf_event *event)
{
	event->fork.pid	 = bswap_32(event->fork.pid);
	event->fork.tid	 = bswap_32(event->fork.tid);
	event->fork.ppid = bswap_32(event->fork.ppid);
	event->fork.ptid = bswap_32(event->fork.ptid);
	event->fork.time = bswap_64(event->fork.time);
}

static void perf_event__read_swap(union perf_event *event)
{
	event->read.pid		 = bswap_32(event->read.pid);
	event->read.tid		 = bswap_32(event->read.tid);
	event->read.value	 = bswap_64(event->read.value);
	event->read.time_enabled = bswap_64(event->read.time_enabled);
	event->read.time_running = bswap_64(event->read.time_running);
	event->read.id		 = bswap_64(event->read.id);
}

/* exported for swapping attributes in file header */
void perf_event__attr_swap(struct perf_event_attr *attr)
{
	attr->type		= bswap_32(attr->type);
	attr->size		= bswap_32(attr->size);
	attr->config		= bswap_64(attr->config);
	attr->sample_period	= bswap_64(attr->sample_period);
	attr->sample_type	= bswap_64(attr->sample_type);
	attr->read_format	= bswap_64(attr->read_format);
	attr->wakeup_events	= bswap_32(attr->wakeup_events);
	attr->bp_type		= bswap_32(attr->bp_type);
	attr->bp_addr		= bswap_64(attr->bp_addr);
	attr->bp_len		= bswap_64(attr->bp_len);
}

static void perf_event__hdr_attr_swap(union perf_event *event)
{
	size_t size;

	perf_event__attr_swap(&event->attr.attr);

	size = event->header.size;
	size -= (void *)&event->attr.id - (void *)event;
	mem_bswap_64(event->attr.id, size);
}

static void perf_event__event_type_swap(union perf_event *event)
{
	event->event_type.event_type.event_id =
		bswap_64(event->event_type.event_type.event_id);
}

static void perf_event__tracing_data_swap(union perf_event *event)
{
	event->tracing_data.size = bswap_32(event->tracing_data.size);
}

typedef void (*perf_event__swap_op)(union perf_event *event);

static perf_event__swap_op perf_event__swap_ops[] = {
	[PERF_RECORD_MMAP]		  = perf_event__mmap_swap,
	[PERF_RECORD_COMM]		  = perf_event__comm_swap,
	[PERF_RECORD_FORK]		  = perf_event__task_swap,
	[PERF_RECORD_EXIT]		  = perf_event__task_swap,
	[PERF_RECORD_LOST]		  = perf_event__all64_swap,
	[PERF_RECORD_READ]		  = perf_event__read_swap,
	[PERF_RECORD_SAMPLE]		  = perf_event__all64_swap,
	[PERF_RECORD_HEADER_ATTR]	  = perf_event__hdr_attr_swap,
	[PERF_RECORD_HEADER_EVENT_TYPE]	  = perf_event__event_type_swap,
	[PERF_RECORD_HEADER_TRACING_DATA] = perf_event__tracing_data_swap,
	[PERF_RECORD_HEADER_BUILD_ID]	  = NULL,
	[PERF_RECORD_HEADER_MAX]	  = NULL,
};

struct sample_queue {
	u64			timestamp;
	u64			file_offset;
	union perf_event	*event;
	struct list_head	list;
};

static void perf_session_free_sample_buffers(struct perf_session *session)
{
	struct ordered_samples *os = &session->ordered_samples;

	while (!list_empty(&os->to_free)) {
		struct sample_queue *sq;

		sq = list_entry(os->to_free.next, struct sample_queue, list);
		list_del(&sq->list);
		free(sq);
	}
}

static int perf_session_deliver_event(struct perf_session *session,
				      union perf_event *event,
				      struct perf_sample *sample,
				      struct perf_event_ops *ops,
				      u64 file_offset);

static void flush_sample_queue(struct perf_session *s,
			       struct perf_event_ops *ops)
{
	struct ordered_samples *os = &s->ordered_samples;
	struct list_head *head = &os->samples;
	struct sample_queue *tmp, *iter;
	struct perf_sample sample;
	u64 limit = os->next_flush;
	u64 last_ts = os->last_sample ? os->last_sample->timestamp : 0ULL;
	int ret;

	if (!ops->ordered_samples || !limit)
		return;

	list_for_each_entry_safe(iter, tmp, head, list) {
		if (iter->timestamp > limit)
			break;

		ret = perf_session__parse_sample(s, iter->event, &sample);
		if (ret)
			pr_err("Can't parse sample, err = %d\n", ret);
		else
			perf_session_deliver_event(s, iter->event, &sample, ops,
						   iter->file_offset);

		os->last_flush = iter->timestamp;
		list_del(&iter->list);
		list_add(&iter->list, &os->sample_cache);
	}

	if (list_empty(head)) {
		os->last_sample = NULL;
	} else if (last_ts <= limit) {
		os->last_sample =
			list_entry(head->prev, struct sample_queue, list);
	}
}

/*
 * When perf record finishes a pass on every buffers, it records this pseudo
 * event.
 * We record the max timestamp t found in the pass n.
 * Assuming these timestamps are monotonic across cpus, we know that if
 * a buffer still has events with timestamps below t, they will be all
 * available and then read in the pass n + 1.
 * Hence when we start to read the pass n + 2, we can safely flush every
 * events with timestamps below t.
 *
 *    ============ PASS n =================
 *       CPU 0         |   CPU 1
 *                     |
 *    cnt1 timestamps  |   cnt2 timestamps
 *          1          |         2
 *          2          |         3
 *          -          |         4  <--- max recorded
 *
 *    ============ PASS n + 1 ==============
 *       CPU 0         |   CPU 1
 *                     |
 *    cnt1 timestamps  |   cnt2 timestamps
 *          3          |         5
 *          4          |         6
 *          5          |         7 <---- max recorded
 *
 *      Flush every events below timestamp 4
 *
 *    ============ PASS n + 2 ==============
 *       CPU 0         |   CPU 1
 *                     |
 *    cnt1 timestamps  |   cnt2 timestamps
 *          6          |         8
 *          7          |         9
 *          -          |         10
 *
 *      Flush every events below timestamp 7
 *      etc...
 */
static int process_finished_round(union perf_event *event __used,
				  struct perf_session *session,
				  struct perf_event_ops *ops)
{
	flush_sample_queue(session, ops);
	session->ordered_samples.next_flush = session->ordered_samples.max_timestamp;

	return 0;
}

/* The queue is ordered by time */
static void __queue_event(struct sample_queue *new, struct perf_session *s)
{
	struct ordered_samples *os = &s->ordered_samples;
	struct sample_queue *sample = os->last_sample;
	u64 timestamp = new->timestamp;
	struct list_head *p;

	os->last_sample = new;

	if (!sample) {
		list_add(&new->list, &os->samples);
		os->max_timestamp = timestamp;
		return;
	}

	/*
	 * last_sample might point to some random place in the list as it's
	 * the last queued event. We expect that the new event is close to
	 * this.
	 */
	if (sample->timestamp <= timestamp) {
		while (sample->timestamp <= timestamp) {
			p = sample->list.next;
			if (p == &os->samples) {
				list_add_tail(&new->list, &os->samples);
				os->max_timestamp = timestamp;
				return;
			}
			sample = list_entry(p, struct sample_queue, list);
		}
		list_add_tail(&new->list, &sample->list);
	} else {
		while (sample->timestamp > timestamp) {
			p = sample->list.prev;
			if (p == &os->samples) {
				list_add(&new->list, &os->samples);
				return;
			}
			sample = list_entry(p, struct sample_queue, list);
		}
		list_add(&new->list, &sample->list);
	}
}

#define MAX_SAMPLE_BUFFER	(64 * 1024 / sizeof(struct sample_queue))

static int perf_session_queue_event(struct perf_session *s, union perf_event *event,
				    struct perf_sample *sample, u64 file_offset)
{
	struct ordered_samples *os = &s->ordered_samples;
	struct list_head *sc = &os->sample_cache;
	u64 timestamp = sample->time;
	struct sample_queue *new;

	if (!timestamp || timestamp == ~0ULL)
		return -ETIME;

	if (timestamp < s->ordered_samples.last_flush) {
		printf("Warning: Timestamp below last timeslice flush\n");
		return -EINVAL;
	}

	if (!list_empty(sc)) {
		new = list_entry(sc->next, struct sample_queue, list);
		list_del(&new->list);
	} else if (os->sample_buffer) {
		new = os->sample_buffer + os->sample_buffer_idx;
		if (++os->sample_buffer_idx == MAX_SAMPLE_BUFFER)
			os->sample_buffer = NULL;
	} else {
		os->sample_buffer = malloc(MAX_SAMPLE_BUFFER * sizeof(*new));
		if (!os->sample_buffer)
			return -ENOMEM;
		list_add(&os->sample_buffer->list, &os->to_free);
		os->sample_buffer_idx = 2;
		new = os->sample_buffer + 1;
	}

	new->timestamp = timestamp;
	new->file_offset = file_offset;
	new->event = event;

	__queue_event(new, s);

	return 0;
}

static void callchain__printf(struct perf_sample *sample)
{
	unsigned int i;

	printf("... chain: nr:%" PRIu64 "\n", sample->callchain->nr);

	for (i = 0; i < sample->callchain->nr; i++)
		printf("..... %2d: %016" PRIx64 "\n",
		       i, sample->callchain->ips[i]);
}

static void perf_session__print_tstamp(struct perf_session *session,
				       union perf_event *event,
				       struct perf_sample *sample)
{
	if (event->header.type != PERF_RECORD_SAMPLE &&
	    !session->sample_id_all) {
		fputs("-1 -1 ", stdout);
		return;
	}

	if ((session->sample_type & PERF_SAMPLE_CPU))
		printf("%u ", sample->cpu);

	if (session->sample_type & PERF_SAMPLE_TIME)
		printf("%" PRIu64 " ", sample->time);
}

static void dump_event(struct perf_session *session, union perf_event *event,
		       u64 file_offset, struct perf_sample *sample)
{
	if (!dump_trace)
		return;

	printf("\n%#" PRIx64 " [%#x]: event: %d\n",
	       file_offset, event->header.size, event->header.type);

	trace_event(event);

	if (sample)
		perf_session__print_tstamp(session, event, sample);

	printf("%#" PRIx64 " [%#x]: PERF_RECORD_%s", file_offset,
	       event->header.size, perf_event__name(event->header.type));
}

static void dump_sample(struct perf_session *session, union perf_event *event,
			struct perf_sample *sample)
{
	if (!dump_trace)
		return;

	printf("(IP, %d): %d/%d: %#" PRIx64 " period: %" PRIu64 "\n",
	       event->header.misc, sample->pid, sample->tid, sample->ip,
	       sample->period);

	if (session->sample_type & PERF_SAMPLE_CALLCHAIN)
		callchain__printf(sample);
}

static int perf_session_deliver_event(struct perf_session *session,
				      union perf_event *event,
				      struct perf_sample *sample,
				      struct perf_event_ops *ops,
				      u64 file_offset)
{
	struct perf_evsel *evsel;

	dump_event(session, event, file_offset, sample);

	switch (event->header.type) {
	case PERF_RECORD_SAMPLE:
		dump_sample(session, event, sample);
		evsel = perf_evlist__id2evsel(session->evlist, sample->id);
		if (evsel == NULL) {
			++session->hists.stats.nr_unknown_id;
			return -1;
		}
		return ops->sample(event, sample, evsel, session);
	case PERF_RECORD_MMAP:
		return ops->mmap(event, sample, session);
	case PERF_RECORD_COMM:
		return ops->comm(event, sample, session);
	case PERF_RECORD_FORK:
		return ops->fork(event, sample, session);
	case PERF_RECORD_EXIT:
		return ops->exit(event, sample, session);
	case PERF_RECORD_LOST:
		return ops->lost(event, sample, session);
	case PERF_RECORD_READ:
		return ops->read(event, sample, session);
	case PERF_RECORD_THROTTLE:
		return ops->throttle(event, sample, session);
	case PERF_RECORD_UNTHROTTLE:
		return ops->unthrottle(event, sample, session);
	default:
		++session->hists.stats.nr_unknown_events;
		return -1;
	}
}

static int perf_session__preprocess_sample(struct perf_session *session,
					   union perf_event *event, struct perf_sample *sample)
{
	if (event->header.type != PERF_RECORD_SAMPLE ||
	    !(session->sample_type & PERF_SAMPLE_CALLCHAIN))
		return 0;

	if (!ip_callchain__valid(sample->callchain, event)) {
		pr_debug("call-chain problem with event, skipping it.\n");
		++session->hists.stats.nr_invalid_chains;
		session->hists.stats.total_invalid_chains += sample->period;
		return -EINVAL;
	}
	return 0;
}

static int perf_session__process_user_event(struct perf_session *session, union perf_event *event,
					    struct perf_event_ops *ops, u64 file_offset)
{
	dump_event(session, event, file_offset, NULL);

	/* These events are processed right away */
	switch (event->header.type) {
	case PERF_RECORD_HEADER_ATTR:
		return ops->attr(event, session);
	case PERF_RECORD_HEADER_EVENT_TYPE:
		return ops->event_type(event, session);
	case PERF_RECORD_HEADER_TRACING_DATA:
		/* setup for reading amidst mmap */
		lseek(session->fd, file_offset, SEEK_SET);
		return ops->tracing_data(event, session);
	case PERF_RECORD_HEADER_BUILD_ID:
		return ops->build_id(event, session);
	case PERF_RECORD_FINISHED_ROUND:
		return ops->finished_round(event, session, ops);
	default:
		return -EINVAL;
	}
}

static int perf_session__process_event(struct perf_session *session,
				       union perf_event *event,
				       struct perf_event_ops *ops,
				       u64 file_offset)
{
	struct perf_sample sample;
	int ret;

	if (session->header.needs_swap &&
	    perf_event__swap_ops[event->header.type])
		perf_event__swap_ops[event->header.type](event);

	if (event->header.type >= PERF_RECORD_HEADER_MAX)
		return -EINVAL;

	hists__inc_nr_events(&session->hists, event->header.type);

	if (event->header.type >= PERF_RECORD_USER_TYPE_START)
		return perf_session__process_user_event(session, event, ops, file_offset);

	/*
	 * For all kernel events we get the sample data
	 */
	ret = perf_session__parse_sample(session, event, &sample);
	if (ret)
		return ret;

	/* Preprocess sample records - precheck callchains */
	if (perf_session__preprocess_sample(session, event, &sample))
		return 0;

	if (ops->ordered_samples) {
		ret = perf_session_queue_event(session, event, &sample,
					       file_offset);
		if (ret != -ETIME)
			return ret;
	}

	return perf_session_deliver_event(session, event, &sample, ops,
					  file_offset);
}

void perf_event_header__bswap(struct perf_event_header *self)
{
	self->type = bswap_32(self->type);
	self->misc = bswap_16(self->misc);
	self->size = bswap_16(self->size);
}

static struct thread *perf_session__register_idle_thread(struct perf_session *self)
{
	struct thread *thread = perf_session__findnew(self, 0);

	if (thread == NULL || thread__set_comm(thread, "swapper")) {
		pr_err("problem inserting idle task.\n");
		thread = NULL;
	}

	return thread;
}

static void perf_session__warn_about_errors(const struct perf_session *session,
					    const struct perf_event_ops *ops)
{
	if (ops->lost == perf_event__process_lost &&
	    session->hists.stats.total_lost != 0) {
		ui__warning("Processed %" PRIu64 " events and LOST %" PRIu64
			    "!\n\nCheck IO/CPU overload!\n\n",
			    session->hists.stats.total_period,
			    session->hists.stats.total_lost);
	}

	if (session->hists.stats.nr_unknown_events != 0) {
		ui__warning("Found %u unknown events!\n\n"
			    "Is this an older tool processing a perf.data "
			    "file generated by a more recent tool?\n\n"
			    "If that is not the case, consider "
			    "reporting to linux-kernel@vger.kernel.org.\n\n",
			    session->hists.stats.nr_unknown_events);
	}

	if (session->hists.stats.nr_unknown_id != 0) {
		ui__warning("%u samples with id not present in the header\n",
			    session->hists.stats.nr_unknown_id);
	}

 	if (session->hists.stats.nr_invalid_chains != 0) {
 		ui__warning("Found invalid callchains!\n\n"
 			    "%u out of %u events were discarded for this reason.\n\n"
 			    "Consider reporting to linux-kernel@vger.kernel.org.\n\n",
 			    session->hists.stats.nr_invalid_chains,
 			    session->hists.stats.nr_events[PERF_RECORD_SAMPLE]);
 	}
}

#define session_done()	(*(volatile int *)(&session_done))
volatile int session_done;

static int __perf_session__process_pipe_events(struct perf_session *self,
					       struct perf_event_ops *ops)
{
	union perf_event event;
	uint32_t size;
	int skip = 0;
	u64 head;
	int err;
	void *p;

	perf_event_ops__fill_defaults(ops);

	head = 0;
more:
	err = readn(self->fd, &event, sizeof(struct perf_event_header));
	if (err <= 0) {
		if (err == 0)
			goto done;

		pr_err("failed to read event header\n");
		goto out_err;
	}

	if (self->header.needs_swap)
		perf_event_header__bswap(&event.header);

	size = event.header.size;
	if (size == 0)
		size = 8;

	p = &event;
	p += sizeof(struct perf_event_header);

	if (size - sizeof(struct perf_event_header)) {
		err = readn(self->fd, p, size - sizeof(struct perf_event_header));
		if (err <= 0) {
			if (err == 0) {
				pr_err("unexpected end of event stream\n");
				goto done;
			}

			pr_err("failed to read event data\n");
			goto out_err;
		}
	}

	if (size == 0 ||
	    (skip = perf_session__process_event(self, &event, ops, head)) < 0) {
		dump_printf("%#" PRIx64 " [%#x]: skipping unknown header type: %d\n",
			    head, event.header.size, event.header.type);
		/*
		 * assume we lost track of the stream, check alignment, and
		 * increment a single u64 in the hope to catch on again 'soon'.
		 */
		if (unlikely(head & 7))
			head &= ~7ULL;

		size = 8;
	}

	head += size;

	if (skip > 0)
		head += skip;

	if (!session_done())
		goto more;
done:
	err = 0;
out_err:
	perf_session__warn_about_errors(self, ops);
	perf_session_free_sample_buffers(self);
	return err;
}

static union perf_event *
fetch_mmaped_event(struct perf_session *session,
		   u64 head, size_t mmap_size, char *buf)
{
	union perf_event *event;

	/*
	 * Ensure we have enough space remaining to read
	 * the size of the event in the headers.
	 */
	if (head + sizeof(event->header) > mmap_size)
		return NULL;

	event = (union perf_event *)(buf + head);

	if (session->header.needs_swap)
		perf_event_header__bswap(&event->header);

	if (head + event->header.size > mmap_size)
		return NULL;

	return event;
}

int __perf_session__process_events(struct perf_session *session,
				   u64 data_offset, u64 data_size,
				   u64 file_size, struct perf_event_ops *ops)
{
	u64 head, page_offset, file_offset, file_pos, progress_next;
	int err, mmap_prot, mmap_flags, map_idx = 0;
	struct ui_progress *progress;
	size_t	page_size, mmap_size;
	char *buf, *mmaps[8];
	union perf_event *event;
	uint32_t size;

	perf_event_ops__fill_defaults(ops);

	page_size = sysconf(_SC_PAGESIZE);

	page_offset = page_size * (data_offset / page_size);
	file_offset = page_offset;
	head = data_offset - page_offset;

	if (data_offset + data_size < file_size)
		file_size = data_offset + data_size;

	progress_next = file_size / 16;
	progress = ui_progress__new("Processing events...", file_size);
	if (progress == NULL)
		return -1;

	mmap_size = session->mmap_window;
	if (mmap_size > file_size)
		mmap_size = file_size;

	memset(mmaps, 0, sizeof(mmaps));

	mmap_prot  = PROT_READ;
	mmap_flags = MAP_SHARED;

	if (session->header.needs_swap) {
		mmap_prot  |= PROT_WRITE;
		mmap_flags = MAP_PRIVATE;
	}
remap:
	buf = mmap(NULL, mmap_size, mmap_prot, mmap_flags, session->fd,
		   file_offset);
	if (buf == MAP_FAILED) {
		pr_err("failed to mmap file\n");
		err = -errno;
		goto out_err;
	}
	mmaps[map_idx] = buf;
	map_idx = (map_idx + 1) & (ARRAY_SIZE(mmaps) - 1);
	file_pos = file_offset + head;

more:
	event = fetch_mmaped_event(session, head, mmap_size, buf);
	if (!event) {
		if (mmaps[map_idx]) {
			munmap(mmaps[map_idx], mmap_size);
			mmaps[map_idx] = NULL;
		}

		page_offset = page_size * (head / page_size);
		file_offset += page_offset;
		head -= page_offset;
		goto remap;
	}

	size = event->header.size;

	if (size == 0 ||
	    perf_session__process_event(session, event, ops, file_pos) < 0) {
		dump_printf("%#" PRIx64 " [%#x]: skipping unknown header type: %d\n",
			    file_offset + head, event->header.size,
			    event->header.type);
		/*
		 * assume we lost track of the stream, check alignment, and
		 * increment a single u64 in the hope to catch on again 'soon'.
		 */
		if (unlikely(head & 7))
			head &= ~7ULL;

		size = 8;
	}

	head += size;
	file_pos += size;

	if (file_pos >= progress_next) {
		progress_next += file_size / 16;
		ui_progress__update(progress, file_pos);
	}

	if (file_pos < file_size)
		goto more;

	err = 0;
	/* do the final flush for ordered samples */
	session->ordered_samples.next_flush = ULLONG_MAX;
	flush_sample_queue(session, ops);
out_err:
	ui_progress__delete(progress);
	perf_session__warn_about_errors(session, ops);
	perf_session_free_sample_buffers(session);
	return err;
}

int perf_session__process_events(struct perf_session *self,
				 struct perf_event_ops *ops)
{
	int err;

	if (perf_session__register_idle_thread(self) == NULL)
		return -ENOMEM;

	if (!self->fd_pipe)
		err = __perf_session__process_events(self,
						     self->header.data_offset,
						     self->header.data_size,
						     self->size, ops);
	else
		err = __perf_session__process_pipe_events(self, ops);

	return err;
}

bool perf_session__has_traces(struct perf_session *self, const char *msg)
{
	if (!(self->sample_type & PERF_SAMPLE_RAW)) {
		pr_err("No trace sample to read. Did you call 'perf %s'?\n", msg);
		return false;
	}

	return true;
}

int perf_session__set_kallsyms_ref_reloc_sym(struct map **maps,
					     const char *symbol_name,
					     u64 addr)
{
	char *bracket;
	enum map_type i;
	struct ref_reloc_sym *ref;

	ref = zalloc(sizeof(struct ref_reloc_sym));
	if (ref == NULL)
		return -ENOMEM;

	ref->name = strdup(symbol_name);
	if (ref->name == NULL) {
		free(ref);
		return -ENOMEM;
	}

	bracket = strchr(ref->name, ']');
	if (bracket)
		*bracket = '\0';

	ref->addr = addr;

	for (i = 0; i < MAP__NR_TYPES; ++i) {
		struct kmap *kmap = map__kmap(maps[i]);
		kmap->ref_reloc_sym = ref;
	}

	return 0;
}

size_t perf_session__fprintf_dsos(struct perf_session *self, FILE *fp)
{
	return __dsos__fprintf(&self->host_machine.kernel_dsos, fp) +
	       __dsos__fprintf(&self->host_machine.user_dsos, fp) +
	       machines__fprintf_dsos(&self->machines, fp);
}

size_t perf_session__fprintf_dsos_buildid(struct perf_session *self, FILE *fp,
					  bool with_hits)
{
	size_t ret = machine__fprintf_dsos_buildid(&self->host_machine, fp, with_hits);
	return ret + machines__fprintf_dsos_buildid(&self->machines, fp, with_hits);
}

size_t perf_session__fprintf_nr_events(struct perf_session *session, FILE *fp)
{
	struct perf_evsel *pos;
	size_t ret = fprintf(fp, "Aggregated stats:\n");

	ret += hists__fprintf_nr_events(&session->hists, fp);

	list_for_each_entry(pos, &session->evlist->entries, node) {
		ret += fprintf(fp, "%s stats:\n", event_name(pos));
		ret += hists__fprintf_nr_events(&pos->hists, fp);
	}

	return ret;
}

struct perf_evsel *perf_session__find_first_evtype(struct perf_session *session,
					      unsigned int type)
{
	struct perf_evsel *pos;

	list_for_each_entry(pos, &session->evlist->entries, node) {
		if (pos->attr.type == type)
			return pos;
	}
	return NULL;
}

void perf_session__print_symbols(union perf_event *event,
				struct perf_sample *sample,
				struct perf_session *session)
{
	struct addr_location al;
	const char *symname, *dsoname;
	struct callchain_cursor *cursor = &session->callchain_cursor;
	struct callchain_cursor_node *node;

	if (perf_event__preprocess_sample(event, session, &al, sample,
					  NULL) < 0) {
		error("problem processing %d event, skipping it.\n",
			event->header.type);
		return;
	}

	if (symbol_conf.use_callchain && sample->callchain) {

		if (perf_session__resolve_callchain(session, al.thread,
						sample->callchain, NULL) != 0) {
			if (verbose)
				error("Failed to resolve callchain. Skipping\n");
			return;
		}
		callchain_cursor_commit(cursor);

		while (1) {
			node = callchain_cursor_current(cursor);
			if (!node)
				break;

			if (node->sym && node->sym->name)
				symname = node->sym->name;
			else
				symname = "";

			if (node->map && node->map->dso && node->map->dso->name)
				dsoname = node->map->dso->name;
			else
				dsoname = "";

			printf("\t%16" PRIx64 " %s (%s)\n", node->ip, symname, dsoname);

			callchain_cursor_advance(cursor);
		}

	} else {
		if (al.sym && al.sym->name)
			symname = al.sym->name;
		else
			symname = "";

		if (al.map && al.map->dso && al.map->dso->name)
			dsoname = al.map->dso->name;
		else
			dsoname = "";

		printf("%16" PRIx64 " %s (%s)", al.addr, symname, dsoname);
	}
}
