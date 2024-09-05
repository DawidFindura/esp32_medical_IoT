#ifndef IOBSERVER_HPP
#define IOBSERVER_HPP

#include "Enums/eExecStatus.hpp"

namespace Interface
{
    class IObserver
    {
    public:

        virtual ~IObserver() = default;

        virtual execStatus update() = 0;
    };
}
#endif // IOBSERVER_HPP