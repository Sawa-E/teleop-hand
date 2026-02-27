#ifndef MOTIONCONTROL_CONTEC_DA_H
#define MOTIONCONTROL_CONTEC_DA_H
#include <sys/io.h>
#include "writer.h"
#include "json_helper.h"
#include "pci_helper.h"

struct contec_da_device_set{
  unsigned int base0;
};

template<typename T>
class contec_da : public writer<T>
{
public:
  const int SUCCESS = 0;
  const int FAIL = 1;
  // method
  contec_da(size_t start_channel, size_t number_of_channel, robot_system *robot, const char *config_file_name)
  :writer<T>(start_channel, number_of_channel, robot, mc::voltage),
  da_output_(writer<T>::number_of_channel_),
  config_file_name_(config_file_name)
  {
    da_resolution_decimal_ = NBitInDecimal(da_resolution_bit_) - 1;
    for(size_t i = 0; i < writer<T>::number_of_channel_; ++i)
    {
      da_output_[i] = da_resolution_decimal_ / 2 ;
    }

    open();
    std::cout << "contec da constructor" << std::endl;
  }

  int open()
  {
    boost::property_tree::ptree pt;
    read_json(config_file_name_.c_str(), pt);
    unsigned int vendor_id = static_cast<unsigned int>(strtol(json_helper<std::string>::get_value_from_json("vendor_id", pt).c_str(), NULL, 16));
    unsigned int device_id = static_cast<unsigned int>(strtol(json_helper<std::string>::get_value_from_json("device_id", pt).c_str(), NULL, 16));
    int board_order = json_helper<int>::get_value_from_json("board_order", pt);
    voltage_range_ = 2.0 * json_helper<double>::get_value_from_json("max_voltage", pt);
    da_pci_.base0 = pci_helper::get_io_port_from_vendor_and_device_id(vendor_id, device_id, board_order);

    std::cout << "[writer] vendor_id: " << vendor_id << std::endl;
    std::cout << "[writer] device_id: " << device_id << std::endl;
    std::cout << "[writer] io port: " << da_pci_.base0 << std::endl;

    initialize();
    return SUCCESS;
  }

  int write()
  {
    writer<T>::write();
    return SUCCESS;
  }

  int write(size_t idx)
  {
    (void)idx;
    std::cerr << "[contec_da] write(size_t idx) is not implemented" << std::endl;
    return FAIL;
  }

  int write_buf()
  {
    for(size_t i = 0; i < writer<T>::number_of_channel_ ; ++i)
    {
      //*(writer<T>::channel_ptr(i)) is joint.data[output][voltage]
      da_output_[i] = (*(writer<T>::channel_ptr(i)) + voltage_range_/2.0)/voltage_range_*da_resolution_decimal_;
    }
    return SUCCESS;
  }

  int flush()
  {
    static unsigned int tmp ;
    static unsigned int ao_flg ;

    //Open Gate for Da Board for sampling
    outl(0x20000001,da_pci_.base0+0x30);
    for(size_t i = 0; i < writer<T>::number_of_channel_ ; ++i)
    {
      tmp = static_cast<unsigned int>(da_output_[i]);
      outw(tmp,da_pci_.base0+0x08);
    }
    outl(0x00000005,da_pci_.base0+0x38);

    int timeout = 10000;
    do
    {
      outl(0x20000000,da_pci_.base0+0x38);
      ao_flg = inl(da_pci_.base0+0x3c) & 0x80000000 ;
      if (--timeout <= 0)
      {
        std::cerr << "[contec_da] flush timeout" << std::endl;
        return FAIL;
      }
    }
    while(ao_flg != 0x80000000) ;
    return SUCCESS;
  }

  void reset()
  {
    reset_da_board(da_pci_.base0) ;
    initial_setting(da_pci_.base0) ;
    callibrate_board(da_pci_.base0) ;

    zero();
  }

  void zero()
  {
    for(size_t i = 0 ;i < writer<T>::number_of_channel_; ++i)
      outw(static_cast<unsigned int>((0 + voltage_range_ / 2.0) * da_resolution_decimal_ / voltage_range_), da_pci_.base0+0x08);

    outl(0x20000001,da_pci_.base0+0x30);
    outl(0x00000005,da_pci_.base0+0x38);
  }

  void initialize()
  {
    std::cout << "Started Da Board" << std::endl;
    iopl(3);// root is needed
    reset();
    std::cout << "DA board reset" << std::endl;
  }

  void close()
  {
    reset();
    std::cout << "conter da is closed" << std::endl;
  }

private:
  T da_resolution_decimal_;
  const int da_resolution_bit_ = 16;
  T voltage_range_;
  contec_da_device_set da_pci_;
  std::vector<unsigned short> da_output_;
  std::string config_file_name_;
  std::string vendor_id_;
  std::string device_id_;

  // method
  T NBitInDecimal(int power){
    int out = 1;
    for(int i = 0; i < power; ++i){
      out *= 2.0;
    }
    return out;
  }
  void initial_setting(int addr){
    //AO更新許可トリガーの定義
    outl(0x00000003, addr + 0x38);
    outw(0x0020, addr + 0x3c);
    outw(0x0180, addr + 0x3e);//汎用コマンド

    //AO更新不許可トリガーの定義
    outl(0x00000003, addr + 0x38);
    outw(0x0022, addr + 0x3c);
    outw(0x0050, addr + 0x3e);

    //AO Clock Condition(innternal, external)
    outl(0x00000003, addr + 0x38);
    outw(0x0024, addr + 0x3c);
    outw(0x0042, addr + 0x3e);//内部クロック

    std::cout << "AO setting" << std::endl;
    //内部クロック
    outl(0x20000003, addr + 0x30);
    outl(0x0000018f, addr + 0x34);//10us

    //単数複数かのチャンネル設定
    outl(0x2000000c, addr + 0x30);
    outw(0x0001, addr + 0x34);//複数

    //チャンネル設定
    outl(0x20000005, addr + 0x30);
    //Open 16ch
    //outw(0x000f, addr + 0x34);//0x0000-0x000fで1ch-16ch
    //outw(0x0007, addr + 0x34);//0x0000-0x0007で1ch-8ch
    outw(writer<T>::number_of_channel_ - 0x0001, addr + 0x34);//0x0000-0x0007で1ch-8ch
  }

  void reset_da_board(int addr){
    //ECU reset
    outl(0x00000000, addr + 0x38);
    //AO reseet
    outl(0x20000000, addr + 0x30);
    //DI reset
    outl(0x30000000, addr + 0x30);
    //DO reset
    outl(0x40000000, addr + 0x30);
    //CNT reset
    outl(0x50000000, addr + 0x30);
    //MEM reset
    outl(0x60000000, addr + 0x30);
  }

  void callibrate_board(int addr){
    int ao_sts;
    int digpot_gain;
    int digpot_offset;
    int eepromdata;
    int timeout;
    //Adjust Output Setting
    outl(0x20000021,addr+0x30);
    outw(0x0200,addr+0x34);
    timeout = 10000;
    do
    {
      ao_sts = inl(addr+0x0c) & 0x00000020;
      if (--timeout <= 0) { std::cerr << "[contec_da] calibrate timeout (1)" << std::endl; return; }
    }
    while(ao_sts != 0x00000000);

    eepromdata = inw(addr+0x36) ;

    digpot_offset = eepromdata & 0x00ff;
    digpot_gain = (eepromdata>>8) & 0x00ff;

    outl(0x20000020,addr+0x30);
    outw(0x0000,addr+0x34);
    outw(digpot_offset,addr+0x36);
    timeout = 10000;
    do
    {
      ao_sts = inl(addr+0x0c) & 0x00000100;
      if (--timeout <= 0) { std::cerr << "[contec_da] calibrate timeout (2)" << std::endl; return; }
    }
    while(ao_sts != 0x00000000);
    //キャリブレーション終了

    outl(0x20000020,addr+0x30);
    outw(0x0001,addr+0x34);
    outw(digpot_gain,addr+0x36);
    timeout = 10000;
    do
    {
      ao_sts = inl(addr+0x0c) & 0x00000100;
      if (--timeout <= 0) { std::cerr << "[contec_da] calibrate timeout (3)" << std::endl; return; }
    }
    while(ao_sts != 0x00000000);
    //キャリブレーション終了
  }
};
#endif //MOTIONCONTROL_CONTEC_DA_H
