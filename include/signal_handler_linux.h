#ifndef _SIGNAL_HANDLER_LINUX_H_
#define _SIGNAL_HANDLER_LINUX_H_

#include <iostream>
#include <signal.h>

namespace SignalHandle{
    void initSignalHandler();
    void callbackSignal(sig_atomic_t sig);
};

#endif
