#include "ch.h"
#include "hal.h"

int16_t pwm = 0;
int16_t increment = 200;


/*===========================================================================*/
/* PWM related.                                                        */
/*===========================================================================*/

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
pwmcb, {
		{ PWM_OUTPUT_ACTIVE_HIGH, NULL },
		{ PWM_OUTPUT_ACTIVE_HIGH, NULL },
		{ PWM_OUTPUT_DISABLED, NULL },
		{ PWM_OUTPUT_DISABLED, NULL }}, 0,
		};

/*
 * Application entry point.
 */
int main(void) {

	halInit();
	chSysInit();

	/* Activate the PWM driver. */
	pwmStart(&PWMD1, &pwmcfg);

	/* Enable the h-bridge. */
	palSetPad(GPIOB, GPIOB_MOTOR_ENABLE);
	palClearPad(GPIOA, GPIOA_MOTOR_D1);
	chThdSleepMilliseconds(500);


	for (;;) {
		pwm += increment;

		if ((pwm >= 4000) || (pwm <= -4000)) {
			increment = -increment;
		}

		palTogglePad(GPIOC, GPIOC_LED);
		chThdSleepMilliseconds(500);
	}

	return CH_SUCCESS;
}
