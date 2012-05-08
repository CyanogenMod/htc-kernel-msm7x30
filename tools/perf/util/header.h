#ifndef __PERF_HEADER_H
#define __PERF_HEADER_H

#include "../../../include/linux/perf_event.h"
#include <sys/types.h>
#include <stdbool.h>
#include "types.h"
#include "event.h"

#include <linux/bitmap.h>

enum {
	HEADER_TRACE_INFO = 1,
	HEADER_BUILD_ID,
	HEADER_LAST_FEATURE,
};

#define HEADER_FEAT_BITS			256

struct perf_file_section {
	u64 offset;
	u64 size;
};

struct perf_file_header {
	u64				magic;
	u64				size;
	u64				attr_size;
	struct perf_file_section	attrs;
	struct perf_file_section	data;
	struct perf_file_section	event_types;
	DECLARE_BITMAP(adds_features, HEADER_FEAT_BITS);
};

struct perf_pipe_file_header {
	u64				magic;
	u64				size;
};

struct perf_header;

int perf_file_header__read(struct perf_file_header *header,
			   struct perf_header *ph, int fd);

struct perf_header {
	int			frozen;
	bool			needs_swap;
	s64			attr_offset;
	u64			data_offset;
	u64			data_size;
	u64			event_offset;
	u64			event_size;
	DECLARE_BITMAP(adds_features, HEADER_FEAT_BITS);
};

struct perf_evlist;

int perf_session__read_header(struct perf_session *session, int fd);
int perf_session__write_header(struct perf_session *session,
			       struct perf_evlist *evlist,
			       int fd, bool at_exit);
int perf_header__write_pipe(int fd);

int perf_header__push_event(u64 id, const char *name);
char *perf_header__find_event(u64 id);

void perf_header__set_feat(struct perf_header *header, int feat);
void perf_header__clear_feat(struct perf_header *header, int feat);
bool perf_header__has_feat(const struct perf_header *header, int feat);

int perf_header__process_sections(struct perf_header *header, int fd,
				  int (*process)(struct perf_file_section *section,
						 struct perf_header *ph,
						 int feat, int fd));

int build_id_cache__add_s(const char *sbuild_id, const char *debugdir,
			  const char *name, bool is_kallsyms);
int build_id_cache__remove_s(const char *sbuild_id, const char *debugdir);

int perf_event__synthesize_attr(struct perf_event_attr *attr, u16 ids, u64 *id,
				perf_event__handler_t process,
				struct perf_session *session);
int perf_session__synthesize_attrs(struct perf_session *session,
				   perf_event__handler_t process);
int perf_event__process_attr(union perf_event *event, struct perf_session *session);

int perf_event__synthesize_event_type(u64 event_id, char *name,
				      perf_event__handler_t process,
				      struct perf_session *session);
int perf_event__synthesize_event_types(perf_event__handler_t process,
				       struct perf_session *session);
int perf_event__process_event_type(union perf_event *event,
				   struct perf_session *session);

int perf_event__synthesize_tracing_data(int fd, struct perf_evlist *evlist,
					perf_event__handler_t process,
					struct perf_session *session);
int perf_event__process_tracing_data(union perf_event *event,
				     struct perf_session *session);

int perf_event__synthesize_build_id(struct dso *pos, u16 misc,
				    perf_event__handler_t process,
				    struct machine *machine,
				    struct perf_session *session);
int perf_event__process_build_id(union perf_event *event,
				 struct perf_session *session);
#endif /* __PERF_HEADER_H */
