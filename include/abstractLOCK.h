#ifndef _ABSTRACT_LOCK_H_
#define _ABSTRACT_LOCK_H_

#include <stdbool.h>

typedef volatile bool abst_lock;

bool abst_lock_try_set(abst_lock *lock);

void abst_lock_spin(abst_lock *lock);

void abst_lock_clear(abst_lock *lock);

#endif //_ABSTRACT_LOCK_H_
