



#include <stdint.h>
#include "stm32f103Driver.h"

#define ACK  0
#define NACK 1
#define DS3231_I2C_ADDRESS 0xD0

//transmit data to RTC
void rtc_DS3231_transmit (uint8_t i2c, uint8_t reg, uint8_t data){
    i2c_start(i2c);
    i2c_send_address(i2c, DS3231_I2C_ADDRESS, 0);
    i2c_send_data(i2c, reg);
    i2c_send_data(i2c, data);
    i2c_stop(i2c);
}

//receive data from RTC
void rtc_DS3231_receive (uint8_t i2c, uint8_t starting_reg, uint8_t * data, uint8_t number_of_registers){
    i2c_start(i2c);                          // Start condition
    i2c_send_address(i2c, DS3231_I2C_ADDRESS, 0);          // Send address with write bit
    i2c_send_data(i2c, starting_reg);        // Set starting register
    i2c_start(i2c);                          // start
    i2c_send_address(i2c, DS3231_I2C_ADDRESS, 1);          // Send address with read bit

    for (int i = 0; i < number_of_registers - 1; i++) {
        data[i] = i2c_receive_data(i2c, ACK);  // Read with ACK
    }
    data[number_of_registers - 1] = i2c_receive_data(i2c, NACK); // Last byte with NACK
    i2c_stop(i2c);                             // Stop condition
}



//converting standard decimal values into DS3231-friendly BCD format
uint8_t decimal_to_bcd (uint8_t decimal_val){
    uint8_t BCD_Val = 0; 
    BCD_Val = (decimal_val/10)<<4;
    BCD_Val |= (decimal_val%10);
    return BCD_Val; 
}

//converting DS3231 BCD format to standard decimal values  
uint8_t bcd_to_decimal (uint8_t bcd_val){
    uint8_t decimal_val = 0; 
    // Extract tens digit from upper nibble and multiply by 10
    decimal_val = (bcd_val >> 4) * 10;
   // Add the units digit from the lower nibble
    decimal_val += (bcd_val & 0x0F);
    return decimal_val; 
}


void update_second (uint8_t i2c, uint8_t sec){
     rtc_DS3231_transmit ( i2c, 0x00 , decimal_to_bcd ( sec));
}

void update_minute (uint8_t i2c, uint8_t minute){
     rtc_DS3231_transmit ( i2c, 0x01 , decimal_to_bcd ( minute));
}

void update_hour (uint8_t i2c, uint8_t hour){
     rtc_DS3231_transmit ( i2c, 0x02 , decimal_to_bcd ( hour));
}

void update_day(uint8_t i2c, uint8_t day){
     rtc_DS3231_transmit ( i2c, 0x04 , decimal_to_bcd ( day));
}

void update_month(uint8_t i2c, uint8_t month){
     rtc_DS3231_transmit ( i2c, 0x05 , decimal_to_bcd ( month));
}

void update_year(uint8_t i2c, uint8_t year){
     rtc_DS3231_transmit ( i2c, 0x06 , decimal_to_bcd ( year));
}

void update_time(uint8_t i2c, uint8_t hour, uint8_t minute, uint8_t second){
    update_second (i2c, second);
    update_minute (i2c, minute);
    update_hour (i2c, hour);
}

void update_date(uint8_t i2c, uint8_t date, uint8_t month, uint8_t year){
    update_day (i2c, date);
    update_month (i2c, month);
    update_year (i2c, year);
}

void get_time_and_date(uint8_t i2c, uint8_t *rtc_data) {
    uint8_t raw_data[7];  // DS3231 returns 7 bytes: sec, min, hr, weekday, date, month, year

    //  Read 7 bytes starting from register 0
    rtc_DS3231_receive(i2c, 0x00, raw_data, 7);

    //  Convert each BCD byte to decimal and store in rtc_data
    for (int i = 0; i < 7; i++) {
        rtc_data[i] = bcd_to_decimal(raw_data[i]);
    }
}


// Converts hour and minute to "HH:MM"
void format_time_string(uint8_t hour, uint8_t minute, char *time_str) {
    time_str[0] = '0' + (hour / 10);
    time_str[1] = '0' + (hour % 10);
    time_str[2] = ':';
    time_str[3] = '0' + (minute / 10);
    time_str[4] = '0' + (minute % 10);
    time_str[5] = '\0';
}


// Converts day, month, year to "DD/MM/YYYY"
void format_date_string(uint8_t day, uint8_t month, uint8_t year, char *date_str) {
    date_str[0] = '0' + (day / 10);
    date_str[1] = '0' + (day % 10);
    date_str[2] = '/';
    date_str[3] = '0' + (month / 10);
    date_str[4] = '0' + (month % 10);
    date_str[5] = '/';

    uint16_t full_year = 2000 + year;
    date_str[6]  = '0' + ((full_year / 1000) % 10);
    date_str[7]  = '0' + ((full_year / 100) % 10);
    date_str[8]  = '0' + ((full_year / 10) % 10);
    date_str[9]  = '0' + (full_year % 10);
    date_str[10] = '\0';
}


