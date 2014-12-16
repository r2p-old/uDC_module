#include "ch.h"
#include "hal.h"

#include "qei.h"

#include <r2p/Middleware.hpp>
#include <r2p/node/led.hpp>
#include <pid.hpp>
#include <r2p/msg/motor.hpp>

#ifndef R2P_MODULE_NAME
#define R2P_MODULE_NAME "uDC"
#endif

static WORKING_AREA(wa_info, 1024);
static r2p::RTCANTransport rtcantra(RTCAND1);

RTCANConfig rtcan_config = { 1000000, 100, 60 };

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME);

/*===========================================================================*/
/* Robot parameters.                                                         */
/*===========================================================================*/
#define _L        0.20f    // Wheel distance [m]
#define _R        0.0775f    // Wheel radius [m]
#define _TICKS 64.0f
#define _RATIO 29.0f
#define _PI 3.14159265359f

#define R2T(r) ((r / (2 * _PI)) * (_TICKS * _RATIO))
#define T2R(t) ((t / (_TICKS * _RATIO)) * (2 * _PI))

#define M2T(m) (m * _TICKS * _RATIO)/(2 * _PI * _R)

PID speed_pid;
int pwm = 0;

/*
 * PWM cyclic callback.
 */
static void pwmcb(PWMDriver *pwmp) {

	(void) pwmp;
	chSysLockFromIsr()
	;
	if (pwm >= 0) {
		pwm_lld_enable_channel(&PWMD1, 0, pwm);
		pwm_lld_enable_channel(&PWMD1, 1, 0);
	} else {
		pwm_lld_enable_channel(&PWMD1, 0, 0);
		pwm_lld_enable_channel(&PWMD1, 1, -pwm);
	}
	chSysUnlockFromIsr();
}

/*
 * PWM configuration.
 */
static PWMConfig pwmcfg = { 18000000, /* 72MHz PWM clock frequency.   */
4096, /* 12-bit PWM, 17KHz frequency. */
pwmcb, { { PWM_OUTPUT_ACTIVE_HIGH, NULL }, { PWM_OUTPUT_ACTIVE_HIGH, NULL }, { PWM_OUTPUT_DISABLED, NULL }, {
	PWM_OUTPUT_DISABLED, NULL } }, 0, };

/*
 * PWM subscriber node
 */

msg_t pwm_node(void * arg) {
	uint8_t index = *(reinterpret_cast<uint8_t *>(arg));
	r2p::Node node("pwm2sub");
	r2p::Subscriber<r2p::PWM2Msg, 5> pwm_sub;
	r2p::PWM2Msg * msgp;

	(void) arg;

	chRegSetThreadName("pwm_node");

	/* Enable the h-bridge. */
	palSetPad(GPIOB, GPIOB_MOTOR_ENABLE); palClearPad(GPIOA, GPIOA_MOTOR_D1);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	node.subscribe(pwm_sub, "pwm");

	for (;;) {
		if (node.spin(r2p::Time::ms(1000))) {
			if (pwm_sub.fetch(msgp)) {
				pwm = msgp->value[index];
				chSysLock()
				;
				if (pwm >= 0) {
					pwm_lld_enable_channel(&PWMD1, 0, msgp->value[index]);
					pwm_lld_enable_channel(&PWMD1, 1, 0);
				} else {
					pwm_lld_enable_channel(&PWMD1, 0, 0);
					pwm_lld_enable_channel(&PWMD1, 1, -msgp->value[index]);
				}
				chSysUnlock();
				pwm_sub.release(*msgp);
			}
		} else {
			// Stop motor if no messages for 1000 ms
			pwm_lld_disable_channel(&PWM_DRIVER, 0);
			pwm_lld_disable_channel(&PWM_DRIVER, 1);
		}
	}
	return CH_SUCCESS;
}

/*
 * PID node
 */

struct pid3_conf {
	uint8_t motor_id;
	float k;
	float ti;
	float td;
	float ts;
};

bool enc_callback(const r2p::tEncoderMsg &msg) {

	pwm = speed_pid.update(msg.delta);

	chSysLock()
	;

	if (pwm > 0) {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, pwm);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);
	} else {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, 0);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, -pwm);
	}
	chSysUnlock();

	return true;
}

msg_t pid3_node(void * arg) {
	pid3_conf * conf = reinterpret_cast<pid3_conf *>(arg);
	r2p::Node node("pid");
	r2p::Subscriber<r2p::Speed3Msg, 5> speed_sub;
	r2p::Subscriber<r2p::tEncoderMsg, 5> enc_sub(enc_callback);
	r2p::Speed3Msg * msgp;
	r2p::Time last_setpoint(0);

	(void) arg;
	chRegSetThreadName("pid");

	speed_pid.config(conf->k, conf->ti, conf->td, conf->ts, -4095.0, 4095.0);

	/* Enable the h-bridge. */
	palSetPad(GPIOB, GPIOB_MOTOR_ENABLE); palClearPad(GPIOA, GPIOA_MOTOR_D1);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	node.subscribe(speed_sub, "speed3");

	switch (conf->motor_id) {
	case 0:
		node.subscribe(enc_sub, "encoder1");
		break;
	case 1:
		node.subscribe(enc_sub, "encoder2");
		break;
	case 2:
		node.subscribe(enc_sub, "encoder3");
		break;
	default:
		node.subscribe(enc_sub, "encoder");
		break;
	}

	for (;;) {
		if (node.spin(r2p::Time::ms(1000))) {
			if (speed_sub.fetch(msgp)) {
				speed_pid.set(msgp->value[conf->motor_id]);
				last_setpoint = r2p::Time::now();
				speed_sub.release(*msgp);
			} else if (r2p::Time::now() - last_setpoint > r2p::Time::ms(1000)) {
				speed_pid.set(0);
			}
		} else {
			// Stop motor if no messages for 1000 ms
			pwm_lld_disable_channel(&PWM_DRIVER, 0);
			pwm_lld_disable_channel(&PWM_DRIVER, 1);
		}
	}

	return CH_SUCCESS;
}

/*
 * QEI publisher node
 */

QEIConfig qeicfg = { QEI_MODE_QUADRATURE, QEI_BOTH_EDGES, QEI_DIRINV_FALSE, };

msg_t encoder_node(void *arg) {
	uint8_t index = *(reinterpret_cast<uint8_t *>(arg));
	r2p::Node node("encoder");
	r2p::Publisher<r2p::tEncoderMsg> enc_pub;
	systime_t time;
	qeidelta_t delta;
	r2p::tEncoderMsg *msgp;

	(void) arg;
	chRegSetThreadName("encoder");

	/* Enable the QEI driver. */
	qeiInit();
	qeiStart(&QEI_DRIVER, &qeicfg);
	qeiEnable (&QEI_DRIVER);

	switch (index) {
	case 0:
		node.advertise(enc_pub, "encoder1");
		break;
	case 1:
		node.advertise(enc_pub, "encoder2");
		break;
	case 2:
		node.advertise(enc_pub, "encoder3");
		break;
	default:
		node.advertise(enc_pub, "encoder");
		break;
	}

	for (;;) {
		time = chTimeNow();
		delta = qeiUpdate(&QEI_DRIVER);

		if (enc_pub.alloc(msgp)) {
			msgp->timestamp.sec = chTimeNow();
			msgp->timestamp.nsec = chTimeNow();
			msgp->delta = T2R(delta);
			enc_pub.publish(*msgp);
		}

		time += MS2ST(50);
		chThdSleepUntil(time);
	}

	return CH_SUCCESS;
}

/*
 * Application entry point.
 */
extern "C" {
int main(void) {

	halInit();
	chSysInit();

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST);
	rtcantra.initialize(rtcan_config);
	r2p::Middleware::instance.start();

	r2p::ledsub_conf ledsub_conf = { "led" };
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, &ledsub_conf);


	uint8_t index = stm32_id8();

	switch (stm32_id8()) {
	case 240: // tilty
		index = 0;
		break;
	case 238:  // tilty
		index = 1;
		break;
	case 14:  // triskar LEFT
		index = 0;
		break;
	case 206:  // triskar RIGHT
		index = 1;
		break;
	case 239:  // triskar REAR
		index = 2;
		break;
	default:
		break;
	}

	if ((stm32_id8() == 240) || (stm32_id8() == 238)) { // tilty
		r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 2, pwm_node, &index);
	} else {
		pid3_conf pid3_conf = {index, 250, 0, 0, 0.01};
		r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 2, pid3_node, &pid3_conf);
	}

	r2p::Thread::create_heap(NULL, THD_WA_SIZE(2048), NORMALPRIO + 1, encoder_node, &index);

	for (;;) {
		r2p::Thread::sleep(r2p::Time::ms(500));
	}

	return CH_SUCCESS;
}
}
