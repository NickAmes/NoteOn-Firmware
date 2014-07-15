 /* Newlib stubs not provided by other parts of the firmware.
  * These functions don't do anything important, they just ensure that the
  * proper symbols are present so the program will compile.
  * 
  * This file is a part of the firmware for the NoteOn Smartpen.
  * Copyright 2014 Nick Ames <nick@fetchmodus.org>. Licensed under the GNU GPLv3.
  * Contains code from the libopencm3 and newlib projects.                    */
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#undef errno
extern int errno;

int _isatty(int file);
int _close(int file);
int _lseek(int file, int ptr, int dir);
caddr_t _sbrk (int incr);
int _fstat (int file, struct stat * st);

int _isatty(int file){
	return 1;
	file = file; /* This suppresses unused parameter warnings. */
}

int _close(int file){
	return -1;
	file = file; /* This suppresses unused parameter warnings. */
}

int _lseek(int file, int ptr, int dir){
	return 0;
	file = file; /* This suppresses unused parameter warnings. */
	ptr = ptr;
	dir = dir;
}

/* From Newlib's syscalls.c */
register char * stack_ptr asm ("sp");

caddr_t _sbrk (int incr){
	extern char   end asm ("end");	/* Defined by the linker.  */
	static char * heap_end;
	char *        prev_heap_end;
	
	if (heap_end == NULL)
		heap_end = & end;
	
	prev_heap_end = heap_end;
	
	if (heap_end + incr > stack_ptr)
	{
		/* Some of the libstdc++-v3 tests rely upon detecting
		 * out of memory errors, so do not abort here.  */
		#if 0
		extern void abort (void);
	
		_write (1, "_sbrk: Heap and stack collision\n", 32);
	
		abort ();
		#else
		errno = ENOMEM;
		return (caddr_t) -1;
		#endif
	}
	
	heap_end += incr;
	
	return (caddr_t) prev_heap_end;
}

int _fstat (int file, struct stat * st){
	memset (st, 0, sizeof (* st));
	st->st_mode = S_IFCHR;
	st->st_blksize = 1024;
	return 0;
	file = file;
}