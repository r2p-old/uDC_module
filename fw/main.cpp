#include "ch.h"
#include "hal.h"

#include <r2p/Middleware.hpp>


#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "uDC"
#endif

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
//	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

 	r2p::Thread::set_priority(r2p::Thread::NORMAL);
	for (;;) {
		palTogglePad(LED_GPIO, LED);
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}
