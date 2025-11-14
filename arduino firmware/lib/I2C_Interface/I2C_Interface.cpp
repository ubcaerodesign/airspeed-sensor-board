#include "I2C_Interface.hpp"

I2C_Interface::I2C_Interface(const uint8_t addr_dev)
{ 
    address_device = addr_dev; 
    raw_data = {0,0,0,0}; //reserve size, reduce growth dynamically 
}
uint8_t I2C_Interface::get_address_device() const
{
    return address_device;
}
std::vector<uint8_t> I2C_Interface::get_raw_data() const 
{
    return raw_data;
}
void I2C_Interface::read()
{
    int number_bytes = Wire.requestFrom(address_device, size_t(4), true); //request 4 bytes
    // pl(number_bytes);
    // int bytes_available = Wire.available();
    // pl(bytes_available);
    while(true) {
        int bytes_available = Wire.available(); 
        if (bytes_available == 0) {
            break;
        }
        uint8_t data_byte = Wire.read();
        // pl("NEW PACKET");
        switch(bytes_available) {
            case 4: 
                raw_data.at(0) = data_byte;
                 pl(raw_data.at(0));
                break; 
            case 3: 
                raw_data.at(1) = data_byte;
                 pl(raw_data.at(1));
                break;
            case 2: 
                raw_data.at(2) = data_byte;
                 pl(raw_data.at(2));
                break;
            case 1: 
                raw_data.at(3) = data_byte;
                 pl(raw_data.at(3));
                break;
            case 0: //is a default case, but for clarity. 
                break;
            default:
                break;
        } 
    }
    // raw_data.at(0)=Wire.read();
    //     pl(Wire.available());
    // raw_data.at(1)=Wire.read();
    //     pl(Wire.available());
    // raw_data.at(2)=Wire.read();
    //     pl(Wire.available());
    // raw_data.at(3)=Wire.read();
    //     pl(Wire.available());
}