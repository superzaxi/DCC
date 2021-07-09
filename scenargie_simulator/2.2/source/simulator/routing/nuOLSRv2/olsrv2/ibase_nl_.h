//
// This file is automatically generated from tempalte/ibase.h
//

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_nl
 * @{
 */

/** Current ibase_nl. */
#define IBASE_NL    (&OLSR->ibase_nl)

////////////////////////////////////////////////////////////////
//
// Iterator
//

/** Gets iterator.
 *
 * @return iterator which points the first tuple
 */
#define  ibase_nl_iter() \
    ((tuple_nl_t*)(tuple_nl_t*)IBASE_NL->next)

/** Checks whether iterator points end of the ibase.
 *
 * @param iter
 * @return true if iter points to the end of ibase
 */
#define ibase_nl_iter_is_end(iter) \
    ((nu_bool_t)((void*)(iter) == (void*)IBASE_NL))

/** Gets next iterator.
 *
 * @param iter
 * @return iterator which points the next tuple
 */
#define ibase_nl_iter_next(iter) \
    ((tuple_nl_t*)(tuple_nl_t*)(iter)->next)

/** Gets the pointer which points end of the ibase.
 *
 * @return iterator which points the end of the ibase
 */
#define ibase_nl_iter_end()    ((tuple_nl_t*)IBASE_NL)

/** Traverses ibase.
 *
 * @param p
 */

/** Traverses the ibase.
 *
 * @param p
 */
#define FOREACH_NL(p)                     \
    for (tuple_nl_t* p = ibase_nl_iter(); \
         !ibase_nl_iter_is_end(p);        \
         p = ibase_nl_iter_next(p))

/** Sets timeout to the tuple.
 *
 * @param tuple
 * @param t
 */
#define tuple_nl_set_timeout(tuple, t)             \
    tuple_time_set_timeout((tuple_time_t*)(tuple), \
        &(OLSR)->ibase_nl_time_list, (t))

////////////////////////////////////////////////////////////////
//
// Information Base
//

/** Clears the change flag.
 */
#define ibase_nl_clear_change_flag() \
    do { IBASE_NL->change = false; } \
    while (0)

/** Checks whether the ibase has been changed.
 */
#define ibase_nl_is_changed() \
    ((nu_bool_t)(IBASE_NL->change))

/** Sets the change flag of the ibase.
 */
#define ibase_nl_change()           \
    do { IBASE_NL->change = true; } \
    while (0)

/** Checks whether the ibase is empty.
 *
 * @return true if the ibase is empty
 */
#define ibase_nl_is_empty()    ((void*)(IBASE_NL) == (void*)IBASE_NL->next)

/** Gets the size of the ibase.
 *
 * @return the size of the ibase
 */
#define ibase_nl_size()        ((IBASE_NL)->n)

/** Gets the top tuple of the ibase.
 *
 * @return the top tuple of the ibase
 */
#define ibase_nl_head()        ((ibase_nl_is_empty()) ? NULL : (IBASE_NL)->next)

/** @} */

}//namespace// //ScenSim-Port://
