#ifndef IBASE_L_LIST__H_
#define IBASE_L_LIST__H_

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_l
 * @{
 */

////////////////////////////////////////////////////////////////
//
// tuple_l_t
//

/**
 * Gets the iterator which points the first element.
 *
 * @return the iterator of the list
 */
static inline tuple_l_t*
_ibase_l_iter(const ibase_l_t* list)
{
    return (tuple_l_t*)list->next;
}

/**
 * Get the reverse iterator which points the first element.
 *
 * @return the iterator of the list
 */
static inline tuple_l_t*
_ibase_l_iter_r(const ibase_l_t* list)
{
    return (tuple_l_t*)list->prev;
}

/**
 * Gets the iterator which points the last(NOT the last element).
 *
 * @param list
 * @return the iterator which points the end of the list.
 */
static inline tuple_l_t*
_ibase_l_iter_end(const ibase_l_t* list)
{
    return (tuple_l_t*)list;
}

/**
 * Checks whether the iterator points the last or not.
 *
 * @param iter
 * @param list
 * @return true if p is the iterator which points to the end of the list
 */
static inline nu_bool_t
_ibase_l_iter_is_end(const tuple_l_t* iter,
        const ibase_l_t* list)
{
    return((void*)iter == (void*)list);
}

/**
 * Gets the next element
 *
 * @param iter
 * @param list
 * @return the iterator which points to the next element
 */
static inline tuple_l_t*
_ibase_l_iter_next(const tuple_l_t* iter, const ibase_l_t* list)
{
    return iter->next;
}

/**
 * Gets the next element.
 *
 * @param iter
 * @param list
 * @return the iterator which points to the next element
 */
static inline tuple_l_t*
_ibase_l_iter_next_r(const tuple_l_t* iter, const ibase_l_t* list)
{
    return iter->prev;
}

/**
 * Inserts the element before the iterator.
 *
 * @param iter
 * @param list
 * @param elt
 */
static inline void
_ibase_l_iter_insert_before(tuple_l_t* iter,
        ibase_l_t* list, tuple_l_t* elt)
{
    assert(elt);

    elt->prev = iter->prev;
    elt->next = iter;
    iter->prev->next = elt;
    iter->prev = elt;

    ++list->n;
}

/**
 * Inserts the element after the iterator.
 *
 * @param iter
 * @param list
 * @param elt
 */
static inline void
_ibase_l_iter_insert_after(tuple_l_t* iter,
        ibase_l_t* list, tuple_l_t* elt)
{
    assert(elt);

    elt->prev = iter;
    elt->next = iter->next;
    iter->next->prev = elt;
    iter->next = elt;

    ++list->n;
}

/**
 * Removes the element which is pointed by the iterator.
 *
 * @param iter
 * @param list
 * @return the iterator which points next element
 */
static inline tuple_l_t*
_ibase_l_iter_remove(tuple_l_t* iter, ibase_l_t* list)
{
    assert(iter != (tuple_l_t*)list);

    tuple_l_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    //iter->next = iter->prev = NULL;
    tuple_l_free(iter);
    return next_p;
}

/**
 * Removes the element which is pointed by the iter.
 *
 * The memory of removed element is NOT freed.
 *
 * @param iter
 * @param list
 * @return the iterator which points next element
 */
static inline tuple_l_t*
_ibase_l_iter_cut(tuple_l_t* iter, ibase_l_t* list)
{
    assert(iter != (tuple_l_t*)list);
    tuple_l_t* next_p = iter->next;
    iter->prev->next = iter->next;
    iter->next->prev = iter->prev;
    --list->n;
    return next_p;
}

/** Traverses all the element of the list.
 *
 * @param iter
 * @param list
 */
#define FOREACH_IBASE_L(iter, list)             \
    for (tuple_l_t* iter = _ibase_l_iter(list); \
         !_ibase_l_iter_is_end(iter, list);     \
         iter = _ibase_l_iter_next(iter, list))

/**
 * Traverse all the element of the list (reverse order).
 *
 * @param iter
 * @param list
 */
#define FOREACH_IBASE_L_R(iter, list)             \
    for (tuple_l_t* iter = _ibase_l_iter_r(list); \
         !_ibase_l_iter_is_end(iter, list);       \
         iter = _ibase_l_iter_next_r(iter, list))

////////////////////////////////////////////////////////////////
//
// ibase_l_t
//

/**
 * @param self
 * @return the size of the list
 */
static inline size_t
_ibase_l_size(const ibase_l_t* self)
{
    return self->n;
}

/**
 * @param self
 * @retval true  if the list is empty
 * @retval false otherwise
 */
static inline nu_bool_t
_ibase_l_is_empty(const ibase_l_t* self)
{
    return self->next == (tuple_l_t*)self;
}

/** Removes the top element.
 *
 * @param self
 */
static inline void
_ibase_l_remove_head(ibase_l_t* self)
{
    if (!_ibase_l_is_empty(self))
        (void)_ibase_l_iter_remove(self->next, self);
}

/** Removes the last element.
 *
 * @param self
 */
static inline void
_ibase_l_remove_tail(ibase_l_t* self)
{
    if (!_ibase_l_is_empty(self))
        (void)_ibase_l_iter_remove(self->prev, self);
}

/** Remove sthe first element of the list, and return it.
 *
 * @param self
 * @return the removed element
 */
static inline tuple_l_t*
_ibase_l_shift(ibase_l_t* self)
{
    if (_ibase_l_is_empty(self))
        return NULL;
    tuple_l_t* result = self->next;
    _ibase_l_iter_cut(self->next, self);
    return result;
}

/** Prepends the element.
 *
 * @param self
 * @param elt
 */
static inline void
_ibase_l_insert_head(ibase_l_t* self, tuple_l_t* elt)
{
    _ibase_l_iter_insert_after((tuple_l_t*)self, self, elt);
}

/** Appends the element.
 *
 * @param self
 * @param elt
 */
static inline void
_ibase_l_insert_tail(ibase_l_t* self, tuple_l_t* elt)
{
    _ibase_l_iter_insert_before((tuple_l_t*)self, self, elt);
}

/** Gets the top element.
 *
 * @param self
 * @return the top element of self
 */
static inline tuple_l_t*
_ibase_l_head(ibase_l_t* self)
{
    if (_ibase_l_is_empty(self))
        return NULL;
    return self->next;
}

/** Removes all the elements
 *
 * @param self
 */
static inline void
_ibase_l_remove_all(ibase_l_t* self)
{
    while (!_ibase_l_is_empty(self))
        (void)_ibase_l_iter_remove(self->prev, self);
}

/** Initializes list.
 *
 * @param self
 */
static inline void
_ibase_l_init(ibase_l_t* self)
{
    self->prev = self->next = (tuple_l_t*)self;
    self->n = 0;
}

/** Destroys the list.
 *
 * @param self
 */
static inline void
_ibase_l_destroy(ibase_l_t* self)
{
    _ibase_l_remove_all(self);
    assert((tuple_l_t*)self == self->next);
    self->next = self->prev = NULL;
}

/** @} */

}//namespace// //ScenSim-Port://

#endif
