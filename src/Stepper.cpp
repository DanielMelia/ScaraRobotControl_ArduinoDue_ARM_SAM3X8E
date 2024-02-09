#include "Stepper.h"

//#define bitSet(value, bit) ((value) |= (1UL << (bit)))
//#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))

// void Stepper::SetMotorPins(Pio* GPIO_x, uint8_t pulse_pin, uint8_t dir_pin, uint8_t ena_pin){
//     GPIO_PLS_PIN = pulse_pin;
//     GPIO_DIR_PIN = dir_pin;
//     GPIO_ENA_PIN = ena_pin;
//     configurePins();
// }

// void Stepper::Step(){
//     //bitToggle(GPIO_PORT, GPIO_PIN);
//     //*GPIO_PORT ^=(1<<GPIO_PIN);
//     GPIO_PORT->PIO_SODR = GPIO_PIN;
// }

// void Stepper::SetDirection(bool dir){

// }

// void Stepper::configurePins() {
//     uint32_t pin_mask;
//     if (GPIO_ENA_PIN == 32){
//         pin_mask = (1 << GPIO_PLS_PIN) | (1 << GPIO_DIR_PIN);
//     }else{
//         pin_mask = (1 << GPIO_PLS_PIN) | (1 << GPIO_DIR_PIN) | (1 << GPIO_ENA_PIN);
//     }
//     // Enable PIO (Peripheral I/O) for the specified pins
//     GPIO_PORT->PIO_PER = pin_mask;
//     GPIO_PORT->PIO_PER |= pin_mask;
//     // Set pins as output
//     GPIO_PORT->PIO_OER = pin_mask;
//     GPIO_PORT->PIO_OER |= pin_mask;
// }