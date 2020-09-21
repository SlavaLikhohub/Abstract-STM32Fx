#include "abstractLOCK.h"
#include <stdint.h>

static inline void __DMB(void)
{
    __asm__ volatile ("DMB\n");
}

static bool __strex(uint8_t value, abst_lock *addr)
{
    uint32_t result;
   __asm__ volatile ("strexb %0, %2, %1" : "=&r" (result), "=Q" (*addr) : "r" ((uint32_t)value) );
   return(result);
}

static inline uint8_t __ldrex(abst_lock *addr)
{
    uint32_t result;
    __asm__ volatile ("ldrexb %0, %1" : "=r" (result) : "Q" (*addr) );
    return ((uint8_t) result);
}

static inline void __clrex(void) {
    __asm__ volatile ("clrex" ::: "memory");
}

/**
 * Try to set a lock
 * 
 * :param lock: Pointer to the lock
 * :return: True if operation success, false otherwise
 */
bool abst_lock_try_set(abst_lock *lock)
{
    if (__ldrex(lock)) { // Locked
        __clrex();
        return false;
    }
    if (__strex(1, lock) == 1) { // Failed
        return false;
    }
    __DMB();
    return true;
}
/**
 * Try to set a lock until success
 * 
 * :param lock: Pointer to the lock
 */
void abst_lock_spin(abst_lock *lock)
{
    while (!abst_lock_try_set(lock));
}

/**
 * Clear a lock
 * 
 * :param lock: Pointer to the lock
 */
void abst_lock_clear(abst_lock *lock)
{
    __DMB();
    *lock = 0;
}
