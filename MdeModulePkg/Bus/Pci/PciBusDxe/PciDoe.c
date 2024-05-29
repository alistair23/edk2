/** @file
  PCIe Data Object Exchange (DOE)

Copyright (c) 2024, Western Digital Corporation or its affiliates.
SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include "PciBus.h"
#include <IndustryStandard/PcieDoeCapbility.h>

/// DOE Header
#define DOE_HEADER1_OFST_VID        0
#define DOE_HEADER1_OFST_TYPE       15

#define DOE_HEADER2_OFST_LEN        0

// DOE Discovery Types
#define PCI_VENDOR_ID_PCI_SIG       0x01
#define PCI_DOE_PROTOCOL_DISCOVERY  0x00

VOID
DoeWaitForNotBusy (
  IN PCI_IO_DEVICE  *PciIoDevice
  )
{
  PCI_EXPRESS_REG_DOE_STATUS      DoeStatus;

  //
  // Read DOE Status
  //
  PciIoDevice->PciIo.Pci.Read (
                               &PciIoDevice->PciIo,
                               EfiPciIoWidthUint32,
                               PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_STATUS_OFFSET,
                               1,
                               &DoeStatus
                               );

  while (DoeStatus.Bits.DoeBusy) {
    PciIoDevice->PciIo.Pci.Read (
                                 &PciIoDevice->PciIo,
                                 EfiPciIoWidthUint32,
                                 PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_STATUS_OFFSET,
                                 1,
                                 &DoeStatus
                                 );
  }

}

VOID
DoeSetGoBit (
  IN PCI_IO_DEVICE  *PciIoDevice
  )
{
  PCI_EXPRESS_REG_DOE_CONTROL      DoeControl;

  //
  // Read DOE Control
  //
  PciIoDevice->PciIo.Pci.Read (
                               &PciIoDevice->PciIo,
                               EfiPciIoWidthUint32,
                               PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_CONTROL_OFFSET,
                               1,
                               &DoeControl
                               );

  // Set Go bit
  DoeControl.Bits.DoeGo = 1;

  DEBUG ((DEBUG_INFO, "Setting DOE Go bit\n"));

  PciIoDevice->PciIo.Pci.Write (
                               &PciIoDevice->PciIo,
                               EfiPciIoWidthUint32,
                               PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_CONTROL_OFFSET,
                               1,
                               &DoeControl
                               );

}

VOID
DoeWaitForStatusDor (
  IN PCI_IO_DEVICE  *PciIoDevice
  )
{
  PCI_EXPRESS_REG_DOE_STATUS      DoeStatus;

  //
  // Read DOE Status
  //
  PciIoDevice->PciIo.Pci.Read (
                               &PciIoDevice->PciIo,
                               EfiPciIoWidthUint32,
                               PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_STATUS_OFFSET,
                               1,
                               &DoeStatus
                               );

  DEBUG ((DEBUG_INFO, "Waiting for DOE Data Object Read\n"));

  while (!DoeStatus.Bits.DataObjectReady) {
    PciIoDevice->PciIo.Pci.Read (
                                 &PciIoDevice->PciIo,
                                 EfiPciIoWidthUint32,
                                 PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_STATUS_OFFSET,
                                 1,
                                 &DoeStatus
                                 );
  }
}

EFI_STATUS
EFIAPI
PciIoDoeTypeCheck(
  IN EFI_PCI_IO_PROTOCOL              *This,
  IN     UINTN                        VendorId,
  IN     UINTN                        Type
  )
{
  EFI_STATUS     Status;
  PCI_IO_DEVICE  *PciIoDevice;
  PCI_EXPRESS_DOE_DISCOVERY_PACKET Packet;
  UINT8 Index = 0;
  UINT16 ResponseVid, ResponseType;
  UINT32 Response;

  PciIoDevice = PCI_IO_DEVICE_FROM_PCI_IO_THIS (This);

  DoeWaitForNotBusy (PciIoDevice);

  Packet.Packet.Header1 = (PCI_DOE_PROTOCOL_DISCOVERY << DOE_HEADER1_OFST_TYPE) | (PCI_VENDOR_ID_PCI_SIG << DOE_HEADER1_OFST_VID);
  Packet.Packet.Header2 = 3 << DOE_HEADER2_OFST_LEN;

  do {
    Packet.Packet.Dw0 = Index;

    DEBUG ((DEBUG_INFO, "Sending DOE Discovery packet\n"));

    // Send the DOE discovery packet
    Status = PciIoDevice->PciIo.Pci.Write (
                                      &PciIoDevice->PciIo,
                                      EfiPciIoWidthUint32,
                                      PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_WRITE_DATA_MAILBOX_OFFSET,
                                      1,
                                      &Packet.Uint32[0]
                                      );
    Status |= PciIoDevice->PciIo.Pci.Write (
                                      &PciIoDevice->PciIo,
                                      EfiPciIoWidthUint32,
                                      PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_WRITE_DATA_MAILBOX_OFFSET,
                                      1,
                                      &Packet.Uint32[1]
                                      );
    Status |= PciIoDevice->PciIo.Pci.Write (
                                      &PciIoDevice->PciIo,
                                      EfiPciIoWidthUint32,
                                      PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_WRITE_DATA_MAILBOX_OFFSET,
                                      1,
                                      &Packet.Uint32[2]
                                      );
    if (EFI_ERROR (Status)) {
      return EFI_UNSUPPORTED;
    }

    // Set DOE Go bit
    DoeSetGoBit (PciIoDevice);

    // Wait for a response
    DoeWaitForStatusDor (PciIoDevice);

    // Read response
    Status = PciIoDevice->PciIo.Pci.Read (
                                      &PciIoDevice->PciIo,
                                      EfiPciIoWidthUint32,
                                      PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_READ_DATA_MAILBOX_OFFSET,
                                      1,
                                      &Response
                                      );
    // Clear the FIFO
    Status |= PciIoDevice->PciIo.Pci.Write (
                                      &PciIoDevice->PciIo,
                                      EfiPciIoWidthUint32,
                                      PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_WRITE_DATA_MAILBOX_OFFSET,
                                      1,
                                      &Response
                                      );
    // Clear the FIFO (second part)
    Status |= PciIoDevice->PciIo.Pci.Write (
                                      &PciIoDevice->PciIo,
                                      EfiPciIoWidthUint32,
                                      PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_WRITE_DATA_MAILBOX_OFFSET,
                                      1,
                                      &Response
                                      );
    if (EFI_ERROR (Status)) {
      return EFI_UNSUPPORTED;
    }

    DEBUG ((DEBUG_INFO, "Response is: 0x%x\n", Response));

    Index = Response >> 24;
    ResponseVid = Response & 0xFFFF;
    ResponseType = (Response >> 16) & 0xFFFF;

    if (VendorId == ResponseVid && Type == ResponseType) {
      return EFI_SUCCESS;
    }

    // Check if Index matches
  } while (Index);

  return RETURN_UNSUPPORTED;
}

EFI_STATUS
EFIAPI
PciIoDoeSend(
  IN EFI_PCI_IO_PROTOCOL              *This,
  IN     UINTN                        *Count,
  IN OUT VOID                         *Buffer
  )
{
  EFI_STATUS     Status;
  PCI_IO_DEVICE  *PciIoDevice;
  UINT8 Index = 0;
  UINTN Operations = *Count / 4;
  UINT32* Data = Buffer;

  PciIoDevice = PCI_IO_DEVICE_FROM_PCI_IO_THIS (This);

  DoeWaitForNotBusy (PciIoDevice);

  while (Operations) {
    Status = PciIoDevice->PciIo.Pci.Write (
                                      &PciIoDevice->PciIo,
                                      EfiPciIoWidthUint32,
                                      PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_WRITE_DATA_MAILBOX_OFFSET,
                                      1,
                                      &Data[Index]
                                      );
    if (EFI_ERROR (Status)) {
      return EFI_UNSUPPORTED;
    }

    Index++;
    Operations--;
  }

  // Set DOE Go bit
  DoeSetGoBit (PciIoDevice);

  return Status;
}

EFI_STATUS
EFIAPI
PciIoDoeRecieve(
  IN EFI_PCI_IO_PROTOCOL              *This,
  IN     UINTN                        *Count,
  IN OUT VOID                         *Buffer
  )
{
  EFI_STATUS     Status;
  PCI_IO_DEVICE  *PciIoDevice;
  PCI_EXPRESS_REG_DOE_STATUS      DoeStatus;
  UINT8 Index = 0;
  UINT32* Data = Buffer;

  PciIoDevice = PCI_IO_DEVICE_FROM_PCI_IO_THIS (This);

  // Wait for a response
  DoeWaitForStatusDor (PciIoDevice);

  //
  // Read DOE Status
  //
  PciIoDevice->PciIo.Pci.Read (
                               &PciIoDevice->PciIo,
                               EfiPciIoWidthUint32,
                               PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_STATUS_OFFSET,
                               1,
                               &DoeStatus
                               );

  DEBUG ((DEBUG_INFO, "Reading data from mailbox\n"));

  while (DoeStatus.Bits.DataObjectReady && (Index * 4) < *Count) {
    // Read response
    Status = PciIoDevice->PciIo.Pci.Read (
                                      &PciIoDevice->PciIo,
                                      EfiPciIoWidthUint32,
                                      PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_READ_DATA_MAILBOX_OFFSET,
                                      1,
                                      &Data[Index]
                                      );
    // Clear the FIFO
    Status |= PciIoDevice->PciIo.Pci.Write (
                                      &PciIoDevice->PciIo,
                                      EfiPciIoWidthUint32,
                                      PciIoDevice->DoeCapabilityOffset + PCI_EXPRESS_REG_DOE_WRITE_DATA_MAILBOX_OFFSET,
                                      1,
                                      &Data[Index]
                                      );
    if (EFI_ERROR (Status)) {
      return EFI_UNSUPPORTED;
    }

    Index++;
  }

  *Count = Index * 4;

  return Status;
}

//
// Pci Io DOE Interface
//
EFI_PCI_IO_DOE  mPciIoDoeInterface = {
  PciIoDoeTypeCheck,
  PciIoDoeSend,
  PciIoDoeRecieve,
};

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
  )
{
  EFI_STATUS  Status;

  Status = LocatePciExpressCapabilityRegBlock (
             PciIoDevice,
             EFI_PCIE_CAPABILITY_ID_DOE,
             &PciIoDevice->DoeCapabilityOffset,
             NULL
             );
  if (EFI_ERROR (Status)) {
    return EFI_UNSUPPORTED;
  }

  DEBUG ((DEBUG_INFO, "Found DOE support!\n"));

  Status = PciIoDoeTypeCheck (&PciIoDevice->PciIo, 0x01, 0x00);
  if (EFI_ERROR (Status)) {
    // DOE Discovery must be supported for DOE
    DEBUG ((DEBUG_INFO, "DOE Discovery is not supported, device is invalid\n"));
    return EFI_UNSUPPORTED;
  }

  DEBUG ((DEBUG_INFO, "Found DOE Discovery support\n"));

  CopyMem (&PciIoDevice->PciDoe, &mPciIoDoeInterface, sizeof (EFI_PCI_IO_DOE));

  return Status;
}
