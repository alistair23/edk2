/** @file
  PCIe Data Object Exchange (DOE)

Copyright (c) 2024, Western Digital Corporation or its affiliates.
SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#ifndef _EFI_PCI_DOE_H_
#define _EFI_PCI_DOE_H_

/**
  Probe if the PCIe device supports DOE and if it does populate the
  EFI_PCI_IO_DOE instance.

  @param PciIoDevice      PCI device instance.

  @retval EFI_UNSUPPORTED PCI Device does not support DOE.
  @retval EFI_SUCCESS     PCI Device does support DOE.

**/
EFI_STATUS
ProbeDoeSupport (
  IN PCI_IO_DEVICE  *PciIoDevice
  );

#endif
