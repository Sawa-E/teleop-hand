#ifndef MOTIONCONTROL_PCI_HELPER_H
#define MOTIONCONTROL_PCI_HELPER_H

#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>
extern "C" {
#include <pci/pci.h>
}

class pci_helper
{
public:
  static unsigned int get_io_port_from_vendor_and_device_id(unsigned int vendor, unsigned int device, int order)
  {
    unsigned int io_port = 0;
    pci_access *p_access = nullptr;

    p_access = pci_alloc();
    pci_init(p_access);

    std::vector<pci_dev*> pci_devs = find_pci_dev(vendor, device, p_access);
    if (pci_devs.size() <= static_cast<size_t>(order))
    {
      std::cerr << "[Error]" << std::endl;
      std::cerr << "\tPCI device could not found by vendor id or device id or the order." << std::endl;
      pci_cleanup(p_access);
      throw std::runtime_error("PCI device not found: vendor/device/order mismatch");
    }
    std::sort(pci_devs.begin(), pci_devs.end(), [](pci_dev *a, pci_dev *b)
    {return a->bus < b->bus;});
    io_port = static_cast<unsigned int>(pci_devs[order]->base_addr[0] - 1);
    pci_cleanup(p_access);
    return io_port;
  }
private:
  static std::vector<pci_dev*> find_pci_dev(unsigned int vendor, unsigned int device, pci_access *p_access)
  {
    std::vector<pci_dev*> ret;

    pci_dev *p_dev = nullptr;

    pci_scan_bus(p_access);
    for (p_dev = p_access->devices; p_dev; p_dev = p_dev->next)
    {
      pci_fill_info(p_dev, PCI_FILL_IDENT | PCI_FILL_BASES);
      if (p_dev->vendor_id == vendor && p_dev->device_id == device)
        ret.push_back(p_dev);
      else
        continue;
    }
    return ret;
  }
};
#endif //MOTIONCONTROL_PCI_HELPER_H
