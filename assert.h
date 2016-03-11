#ifndef _ASSERT_H_INCLUDED_
#define _ASSERT_H_INCLUDED_

#ifdef NDEBUG
#	define assert(expr)	(void (0))
#else
#	define assert(x)   ((x)?((void) 0):_assert_failed (#x, __FILE__, __LINE__))
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern void _assert_failed (const char *assertion, const char *file, unsigned int line);
int isIntContext(void);
void spinDelayUs(uint32_t us);
void spinDelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif


