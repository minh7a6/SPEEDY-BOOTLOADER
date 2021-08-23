#include "main.h"
#include <stdlib.h>
#include "tlsf.h"
#define TLSF_BUFFER     (0x1400)

tlsf_t _ins = NULL;
static __attribute__((aligned(4))) uint8_t memPool[TLSF_BUFFER];

void heapInit(void) 
{
    _ins = tlsf_create_with_pool(memPool, sizeof(memPool));
    if(_ins == NULL) Error_Handler();
}

void *malloc(size_t sz) 
{
    void *p = NULL;
    p = tlsf_malloc(_ins, sz);
    return p;
}

void free(void *p) 
{
    tlsf_free(_ins, p);
}

void *memalign (size_t boundary, size_t size)
{
    void *ptr = NULL;
    ptr = tlsf_memalign(_ins, boundary, size);
    return ptr;
}

void *realloc( void *ptr, size_t new_size )
{
    return tlsf_realloc(_ins, ptr, new_size);
    
}