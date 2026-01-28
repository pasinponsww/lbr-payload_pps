// Minimal syscalls.c for STM32 bare-metal (newlib stubs)
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <unistd.h>

extern int errno;

int _close(int fd) { (void)fd; errno = EBADF; return -1; }
int _fstat(int fd, struct stat *st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
off_t _lseek(int fd, off_t offset, int whence) { (void)fd; (void)offset; (void)whence; return 0; }
int _open(const char *path, int flags, ...) { (void)path; (void)flags; errno = ENOSYS; return -1; }
ssize_t _read(int fd, void *buf, size_t count) { (void)fd; (void)buf; (void)count; return 0; }
ssize_t _write(int fd, const void *buf, size_t count) { (void)fd; (void)buf; (void)count; return count; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
void _exit(int status) { (void)status; while (1) {} }
void *_sbrk(ptrdiff_t incr) { extern char _end; static char *heap_end; char *prev_heap_end; if (heap_end == 0) heap_end = &_end; prev_heap_end = heap_end; heap_end += incr; return (void *)prev_heap_end; }
