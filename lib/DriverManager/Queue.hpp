#ifndef CQUEUE_H
#define CQUEUE_H

#include <queue>
#include <pthread.h>
#include <unistd.h>

/**
 * @brief Namespace Utils
 * 
 * @namespace Utils
 * 
 */
namespace Utils
{

    /**
     * @brief CQueue class template for creating concurrent queues
     * 
     * @class CQueue
     */
    template <typename T>
    class CQueue
    {
    public:
        /**
         * @brief Construct a new CQueue object
         * 
         */
        CQueue()
        {
            pthread_mutex_init(&m_mutex, NULL);
            pthread_cond_init(&m_cond, NULL);
        }
        /**
         * @brief Destroy CQueue object
         * 
         */
        ~CQueue()
        {
            pthread_mutex_destroy(&m_mutex);
            pthread_cond_destroy(&m_cond);
        }
        /**
         * @brief Pop the first element from the queue
         * 
         * @return T first element in the queue
         * 
         */
        T pop()
        {
            pthread_mutex_lock(&m_mutex);
            while (m_quee.empty())
            {
                pthread_cond_wait(&m_cond, &m_mutex);
            }
            T& item = m_quee.front();
            m_quee.pop();
            pthread_mutex_unlock(&m_mutex);
            return item;
        }
        /**
         * @brief Pop the first element from the queue and assign it to argument
         * 
         * @param a_item T&
         * 
         */
        void pop(T& a_item)
        {
            pthread_mutex_lock(&m_mutex);
            while (m_quee.empty())
            {
                pthread_cond_wait(&m_cond, &m_mutex);
            }
            a_item = m_quee.front();
            m_quee.pop();
            pthread_mutex_unlock(&m_mutex);
        }
        /**
         * @brief Push an element to the queue
         * 
         * @param a_item const T&
         * 
         */
        void push(const T& a_item)
        {
            pthread_mutex_lock(&m_mutex);
            m_quee.push(a_item);
            pthread_mutex_unlock(&m_mutex);
            pthread_cond_signal(&m_cond);
        }
        /**
         * @brief Check if the queue is empty
         * 
         * @return bool
         * 
         */
        bool isEmpty()
        {
            pthread_mutex_lock(&m_mutex);
            bool empty = m_quee.empty();
            pthread_mutex_unlock(&m_mutex);

            return empty;
        }

    private:
        std::queue<T> m_quee;
        pthread_mutex_t m_mutex;
        pthread_cond_t m_cond;
    };
}// ns Utils

#endif // CQUEUE_H