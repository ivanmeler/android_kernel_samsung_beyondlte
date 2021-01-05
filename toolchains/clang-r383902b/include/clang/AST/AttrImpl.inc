/*===- TableGen'erated file -------------------------------------*- C++ -*-===*\
|*                                                                            *|
|* Attribute classes' member function definitions                             *|
|*                                                                            *|
|* Automatically generated file, do not edit!                                 *|
|*                                                                            *|
\*===----------------------------------------------------------------------===*/

AArch64VectorPcsAttr *AArch64VectorPcsAttr::clone(ASTContext &C) const {
  auto *A = new (C) AArch64VectorPcsAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AArch64VectorPcsAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((aarch64_vector_pcs))";
    break;
  }
  case 1 : {
    OS << " [[clang::aarch64_vector_pcs]]";
    break;
  }
  case 2 : {
    OS << " [[clang::aarch64_vector_pcs]]";
    break;
  }
}
}

const char *AArch64VectorPcsAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "aarch64_vector_pcs";
  case 1:
    return "aarch64_vector_pcs";
  case 2:
    return "aarch64_vector_pcs";
  }
}

AMDGPUFlatWorkGroupSizeAttr *AMDGPUFlatWorkGroupSizeAttr::clone(ASTContext &C) const {
  auto *A = new (C) AMDGPUFlatWorkGroupSizeAttr(C, *this, min, max);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AMDGPUFlatWorkGroupSizeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((amdgpu_flat_work_group_size(" << getMin() << ", " << getMax() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::amdgpu_flat_work_group_size(" << getMin() << ", " << getMax() << ")]]";
    break;
  }
}
}

const char *AMDGPUFlatWorkGroupSizeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "amdgpu_flat_work_group_size";
  case 1:
    return "amdgpu_flat_work_group_size";
  }
}

AMDGPUNumSGPRAttr *AMDGPUNumSGPRAttr::clone(ASTContext &C) const {
  auto *A = new (C) AMDGPUNumSGPRAttr(C, *this, numSGPR);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AMDGPUNumSGPRAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((amdgpu_num_sgpr(" << getNumSGPR() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::amdgpu_num_sgpr(" << getNumSGPR() << ")]]";
    break;
  }
}
}

const char *AMDGPUNumSGPRAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "amdgpu_num_sgpr";
  case 1:
    return "amdgpu_num_sgpr";
  }
}

AMDGPUNumVGPRAttr *AMDGPUNumVGPRAttr::clone(ASTContext &C) const {
  auto *A = new (C) AMDGPUNumVGPRAttr(C, *this, numVGPR);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AMDGPUNumVGPRAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((amdgpu_num_vgpr(" << getNumVGPR() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::amdgpu_num_vgpr(" << getNumVGPR() << ")]]";
    break;
  }
}
}

const char *AMDGPUNumVGPRAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "amdgpu_num_vgpr";
  case 1:
    return "amdgpu_num_vgpr";
  }
}

AMDGPUWavesPerEUAttr *AMDGPUWavesPerEUAttr::clone(ASTContext &C) const {
  auto *A = new (C) AMDGPUWavesPerEUAttr(C, *this, min, max);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AMDGPUWavesPerEUAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((amdgpu_waves_per_eu(" << getMin() << ", " << getMax() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::amdgpu_waves_per_eu(" << getMin() << ", " << getMax() << ")]]";
    break;
  }
}
}

const char *AMDGPUWavesPerEUAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "amdgpu_waves_per_eu";
  case 1:
    return "amdgpu_waves_per_eu";
  }
}

ARMInterruptAttr *ARMInterruptAttr::clone(ASTContext &C) const {
  auto *A = new (C) ARMInterruptAttr(C, *this, interrupt);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ARMInterruptAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((interrupt(\"" << ARMInterruptAttr::ConvertInterruptTypeToStr(getInterrupt()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::interrupt(\"" << ARMInterruptAttr::ConvertInterruptTypeToStr(getInterrupt()) << "\")]]";
    break;
  }
}
}

const char *ARMInterruptAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "interrupt";
  case 1:
    return "interrupt";
  }
}

AVRInterruptAttr *AVRInterruptAttr::clone(ASTContext &C) const {
  auto *A = new (C) AVRInterruptAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AVRInterruptAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((interrupt))";
    break;
  }
  case 1 : {
    OS << " [[gnu::interrupt]]";
    break;
  }
}
}

const char *AVRInterruptAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "interrupt";
  case 1:
    return "interrupt";
  }
}

AVRSignalAttr *AVRSignalAttr::clone(ASTContext &C) const {
  auto *A = new (C) AVRSignalAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AVRSignalAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((signal))";
    break;
  }
  case 1 : {
    OS << " [[gnu::signal]]";
    break;
  }
}
}

const char *AVRSignalAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "signal";
  case 1:
    return "signal";
  }
}

AbiTagAttr *AbiTagAttr::clone(ASTContext &C) const {
  auto *A = new (C) AbiTagAttr(C, *this, tags_, tags_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AbiTagAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((abi_tag(";
  bool isFirst = true;
  for (const auto &Val : tags()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << Val << "\"";
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::abi_tag(";
  bool isFirst = true;
  for (const auto &Val : tags()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << Val << "\"";
  }
  OS << ")]]";
    break;
  }
}
}

const char *AbiTagAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "abi_tag";
  case 1:
    return "abi_tag";
  }
}

AcquireCapabilityAttr *AcquireCapabilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) AcquireCapabilityAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AcquireCapabilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((acquire_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::acquire_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " __attribute__((acquire_shared_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 3 : {
    OS << " [[clang::acquire_shared_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 4 : {
    OS << " __attribute__((exclusive_lock_function(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 5 : {
    OS << " __attribute__((shared_lock_function(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
}
}

const char *AcquireCapabilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "acquire_capability";
  case 1:
    return "acquire_capability";
  case 2:
    return "acquire_shared_capability";
  case 3:
    return "acquire_shared_capability";
  case 4:
    return "exclusive_lock_function";
  case 5:
    return "shared_lock_function";
  }
}

AcquireHandleAttr *AcquireHandleAttr::clone(ASTContext &C) const {
  auto *A = new (C) AcquireHandleAttr(C, *this, getHandleType());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AcquireHandleAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((acquire_handle(\"" << getHandleType() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::acquire_handle(\"" << getHandleType() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::acquire_handle(\"" << getHandleType() << "\")]]";
    break;
  }
}
}

const char *AcquireHandleAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "acquire_handle";
  case 1:
    return "acquire_handle";
  case 2:
    return "acquire_handle";
  }
}

AcquiredAfterAttr *AcquiredAfterAttr::clone(ASTContext &C) const {
  auto *A = new (C) AcquiredAfterAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AcquiredAfterAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((acquired_after(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
}
}

const char *AcquiredAfterAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "acquired_after";
  }
}

AcquiredBeforeAttr *AcquiredBeforeAttr::clone(ASTContext &C) const {
  auto *A = new (C) AcquiredBeforeAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AcquiredBeforeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((acquired_before(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
}
}

const char *AcquiredBeforeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "acquired_before";
  }
}

AddressSpaceAttr *AddressSpaceAttr::clone(ASTContext &C) const {
  auto *A = new (C) AddressSpaceAttr(C, *this, addressSpace);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AddressSpaceAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((address_space(" << getAddressSpace() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::address_space(" << getAddressSpace() << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::address_space(" << getAddressSpace() << ")]]";
    break;
  }
}
}

const char *AddressSpaceAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "address_space";
  case 1:
    return "address_space";
  case 2:
    return "address_space";
  }
}

AliasAttr *AliasAttr::clone(ASTContext &C) const {
  auto *A = new (C) AliasAttr(C, *this, getAliasee());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AliasAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((alias(\"" << getAliasee() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::alias(\"" << getAliasee() << "\")]]";
    break;
  }
}
}

const char *AliasAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "alias";
  case 1:
    return "alias";
  }
}

AlignMac68kAttr *AlignMac68kAttr::clone(ASTContext &C) const {
  auto *A = new (C) AlignMac68kAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AlignMac68kAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *AlignMac68kAttr::getSpelling() const {
  return "(No spelling)";
}

AlignValueAttr *AlignValueAttr::clone(ASTContext &C) const {
  auto *A = new (C) AlignValueAttr(C, *this, alignment);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AlignValueAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((align_value(" << getAlignment() << ")))";
    break;
  }
}
}

const char *AlignValueAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "align_value";
  }
}

bool AlignedAttr::isAlignmentDependent() const {
  if (isalignmentExpr)
    return alignmentExpr && (alignmentExpr->isValueDependent() || alignmentExpr->isTypeDependent());
  else
    return alignmentType->getType()->isDependentType();
}
unsigned AlignedAttr::getAlignment(ASTContext &Ctx) const {
  assert(!isAlignmentDependent());
  if (isalignmentExpr)
    return alignmentExpr ? alignmentExpr->EvaluateKnownConstInt(Ctx).getZExtValue() * Ctx.getCharWidth() : Ctx.getTargetDefaultAlignForAttributeAligned();
  else
    return 0; // FIXME
}
AlignedAttr *AlignedAttr::clone(ASTContext &C) const {
  auto *A = new (C) AlignedAttr(C, *this, isalignmentExpr, isalignmentExpr ? static_cast<void*>(alignmentExpr) : alignmentType);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AlignedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((aligned";
    unsigned TrailingOmittedArgs = 0;
    if (!isalignmentExpr || !alignmentExpr)
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!isalignmentExpr || !alignmentExpr)) {
      OS << "";
    alignmentExpr->printPretty(OS, nullptr, Policy);
    OS << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << "))";
    break;
  }
  case 1 : {
    OS << " [[gnu::aligned";
    unsigned TrailingOmittedArgs = 0;
    if (!isalignmentExpr || !alignmentExpr)
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!isalignmentExpr || !alignmentExpr)) {
      OS << "";
    alignmentExpr->printPretty(OS, nullptr, Policy);
    OS << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << "]]";
    break;
  }
  case 2 : {
    OS << " __declspec(align";
    unsigned TrailingOmittedArgs = 0;
    if (!isalignmentExpr || !alignmentExpr)
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!isalignmentExpr || !alignmentExpr)) {
      OS << "";
    alignmentExpr->printPretty(OS, nullptr, Policy);
    OS << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << ")";
    break;
  }
  case 3 : {
    OS << " alignas";
    unsigned TrailingOmittedArgs = 0;
    if (!isalignmentExpr || !alignmentExpr)
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!isalignmentExpr || !alignmentExpr)) {
      OS << "";
    alignmentExpr->printPretty(OS, nullptr, Policy);
    OS << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << "";
    break;
  }
  case 4 : {
    OS << " _Alignas";
    unsigned TrailingOmittedArgs = 0;
    if (!isalignmentExpr || !alignmentExpr)
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!isalignmentExpr || !alignmentExpr)) {
      OS << "";
    alignmentExpr->printPretty(OS, nullptr, Policy);
    OS << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << "";
    break;
  }
}
}

const char *AlignedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "aligned";
  case 1:
    return "aligned";
  case 2:
    return "align";
  case 3:
    return "alignas";
  case 4:
    return "_Alignas";
  }
}

AllocAlignAttr *AllocAlignAttr::clone(ASTContext &C) const {
  auto *A = new (C) AllocAlignAttr(C, *this, paramIndex);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AllocAlignAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((alloc_align(" << getParamIndex().getSourceIndex() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::alloc_align(" << getParamIndex().getSourceIndex() << ")]]";
    break;
  }
}
}

const char *AllocAlignAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "alloc_align";
  case 1:
    return "alloc_align";
  }
}

AllocSizeAttr *AllocSizeAttr::clone(ASTContext &C) const {
  auto *A = new (C) AllocSizeAttr(C, *this, elemSizeParam, numElemsParam);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AllocSizeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((alloc_size";
    unsigned TrailingOmittedArgs = 0;
    if (!getNumElemsParam().isValid())
      ++TrailingOmittedArgs;
    OS << "(" << getElemSizeParam().getSourceIndex() << "";
    if (1 < 2 - TrailingOmittedArgs)
      OS << ", ";
    OS << "";
    if (!(!getNumElemsParam().isValid())) {
      OS << "" << getNumElemsParam().getSourceIndex() << "";
    }
    OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::alloc_size";
    unsigned TrailingOmittedArgs = 0;
    if (!getNumElemsParam().isValid())
      ++TrailingOmittedArgs;
    OS << "(" << getElemSizeParam().getSourceIndex() << "";
    if (1 < 2 - TrailingOmittedArgs)
      OS << ", ";
    OS << "";
    if (!(!getNumElemsParam().isValid())) {
      OS << "" << getNumElemsParam().getSourceIndex() << "";
    }
    OS << ")]]";
    break;
  }
}
}

const char *AllocSizeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "alloc_size";
  case 1:
    return "alloc_size";
  }
}

AlwaysDestroyAttr *AlwaysDestroyAttr::clone(ASTContext &C) const {
  auto *A = new (C) AlwaysDestroyAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AlwaysDestroyAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((always_destroy))";
    break;
  }
  case 1 : {
    OS << " [[clang::always_destroy]]";
    break;
  }
}
}

const char *AlwaysDestroyAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "always_destroy";
  case 1:
    return "always_destroy";
  }
}

AlwaysInlineAttr *AlwaysInlineAttr::clone(ASTContext &C) const {
  auto *A = new (C) AlwaysInlineAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AlwaysInlineAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((always_inline))";
    break;
  }
  case 1 : {
    OS << " [[gnu::always_inline]]";
    break;
  }
  case 2 : {
    OS << " __forceinline";
    break;
  }
}
}

const char *AlwaysInlineAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "always_inline";
  case 1:
    return "always_inline";
  case 2:
    return "__forceinline";
  }
}

AnalyzerNoReturnAttr *AnalyzerNoReturnAttr::clone(ASTContext &C) const {
  auto *A = new (C) AnalyzerNoReturnAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AnalyzerNoReturnAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((analyzer_noreturn))";
    break;
  }
}
}

const char *AnalyzerNoReturnAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "analyzer_noreturn";
  }
}

AnnotateAttr *AnnotateAttr::clone(ASTContext &C) const {
  auto *A = new (C) AnnotateAttr(C, *this, getAnnotation());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AnnotateAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((annotate(\"" << getAnnotation() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::annotate(\"" << getAnnotation() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::annotate(\"" << getAnnotation() << "\")]]";
    break;
  }
}
}

const char *AnnotateAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "annotate";
  case 1:
    return "annotate";
  case 2:
    return "annotate";
  }
}

AnyX86InterruptAttr *AnyX86InterruptAttr::clone(ASTContext &C) const {
  auto *A = new (C) AnyX86InterruptAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AnyX86InterruptAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((interrupt))";
    break;
  }
  case 1 : {
    OS << " [[gnu::interrupt]]";
    break;
  }
}
}

const char *AnyX86InterruptAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "interrupt";
  case 1:
    return "interrupt";
  }
}

AnyX86NoCallerSavedRegistersAttr *AnyX86NoCallerSavedRegistersAttr::clone(ASTContext &C) const {
  auto *A = new (C) AnyX86NoCallerSavedRegistersAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AnyX86NoCallerSavedRegistersAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((no_caller_saved_registers))";
    break;
  }
  case 1 : {
    OS << " [[gnu::no_caller_saved_registers]]";
    break;
  }
}
}

const char *AnyX86NoCallerSavedRegistersAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_caller_saved_registers";
  case 1:
    return "no_caller_saved_registers";
  }
}

AnyX86NoCfCheckAttr *AnyX86NoCfCheckAttr::clone(ASTContext &C) const {
  auto *A = new (C) AnyX86NoCfCheckAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AnyX86NoCfCheckAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((nocf_check))";
    break;
  }
  case 1 : {
    OS << " [[gnu::nocf_check]]";
    break;
  }
}
}

const char *AnyX86NoCfCheckAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "nocf_check";
  case 1:
    return "nocf_check";
  }
}

ArcWeakrefUnavailableAttr *ArcWeakrefUnavailableAttr::clone(ASTContext &C) const {
  auto *A = new (C) ArcWeakrefUnavailableAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ArcWeakrefUnavailableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_arc_weak_reference_unavailable))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_arc_weak_reference_unavailable]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_arc_weak_reference_unavailable]]";
    break;
  }
}
}

const char *ArcWeakrefUnavailableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_arc_weak_reference_unavailable";
  case 1:
    return "objc_arc_weak_reference_unavailable";
  case 2:
    return "objc_arc_weak_reference_unavailable";
  }
}

ArgumentWithTypeTagAttr *ArgumentWithTypeTagAttr::clone(ASTContext &C) const {
  auto *A = new (C) ArgumentWithTypeTagAttr(C, *this, argumentKind, argumentIdx, typeTagIdx, isPointer);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ArgumentWithTypeTagAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((argument_with_type_tag(" << (getArgumentKind() ? getArgumentKind()->getName() : "") << ", " << getArgumentIdx().getSourceIndex() << ", " << getTypeTagIdx().getSourceIndex() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::argument_with_type_tag(" << (getArgumentKind() ? getArgumentKind()->getName() : "") << ", " << getArgumentIdx().getSourceIndex() << ", " << getTypeTagIdx().getSourceIndex() << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::argument_with_type_tag(" << (getArgumentKind() ? getArgumentKind()->getName() : "") << ", " << getArgumentIdx().getSourceIndex() << ", " << getTypeTagIdx().getSourceIndex() << ")]]";
    break;
  }
  case 3 : {
    OS << " __attribute__((pointer_with_type_tag(" << (getArgumentKind() ? getArgumentKind()->getName() : "") << ", " << getArgumentIdx().getSourceIndex() << ", " << getTypeTagIdx().getSourceIndex() << ")))";
    break;
  }
  case 4 : {
    OS << " [[clang::pointer_with_type_tag(" << (getArgumentKind() ? getArgumentKind()->getName() : "") << ", " << getArgumentIdx().getSourceIndex() << ", " << getTypeTagIdx().getSourceIndex() << ")]]";
    break;
  }
  case 5 : {
    OS << " [[clang::pointer_with_type_tag(" << (getArgumentKind() ? getArgumentKind()->getName() : "") << ", " << getArgumentIdx().getSourceIndex() << ", " << getTypeTagIdx().getSourceIndex() << ")]]";
    break;
  }
}
}

const char *ArgumentWithTypeTagAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "argument_with_type_tag";
  case 1:
    return "argument_with_type_tag";
  case 2:
    return "argument_with_type_tag";
  case 3:
    return "pointer_with_type_tag";
  case 4:
    return "pointer_with_type_tag";
  case 5:
    return "pointer_with_type_tag";
  }
}

ArmMveAliasAttr *ArmMveAliasAttr::clone(ASTContext &C) const {
  auto *A = new (C) ArmMveAliasAttr(C, *this, builtinName);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ArmMveAliasAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((__clang_arm_mve_alias(" << (getBuiltinName() ? getBuiltinName()->getName() : "") << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::__clang_arm_mve_alias(" << (getBuiltinName() ? getBuiltinName()->getName() : "") << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::__clang_arm_mve_alias(" << (getBuiltinName() ? getBuiltinName()->getName() : "") << ")]]";
    break;
  }
}
}

const char *ArmMveAliasAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__clang_arm_mve_alias";
  case 1:
    return "__clang_arm_mve_alias";
  case 2:
    return "__clang_arm_mve_alias";
  }
}

ArmMveStrictPolymorphismAttr *ArmMveStrictPolymorphismAttr::clone(ASTContext &C) const {
  auto *A = new (C) ArmMveStrictPolymorphismAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ArmMveStrictPolymorphismAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((__clang_arm_mve_strict_polymorphism))";
    break;
  }
  case 1 : {
    OS << " [[clang::__clang_arm_mve_strict_polymorphism]]";
    break;
  }
  case 2 : {
    OS << " [[clang::__clang_arm_mve_strict_polymorphism]]";
    break;
  }
}
}

const char *ArmMveStrictPolymorphismAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__clang_arm_mve_strict_polymorphism";
  case 1:
    return "__clang_arm_mve_strict_polymorphism";
  case 2:
    return "__clang_arm_mve_strict_polymorphism";
  }
}

ArtificialAttr *ArtificialAttr::clone(ASTContext &C) const {
  auto *A = new (C) ArtificialAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ArtificialAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((artificial))";
    break;
  }
  case 1 : {
    OS << " [[gnu::artificial]]";
    break;
  }
}
}

const char *ArtificialAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "artificial";
  case 1:
    return "artificial";
  }
}

AsmLabelAttr *AsmLabelAttr::clone(ASTContext &C) const {
  auto *A = new (C) AsmLabelAttr(C, *this, getLabel(), isLiteralLabel);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AsmLabelAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " asm(\"" << getLabel() << "\")";
    break;
  }
  case 1 : {
    OS << " __asm__(\"" << getLabel() << "\")";
    break;
  }
}
}

const char *AsmLabelAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "asm";
  case 1:
    return "__asm__";
  }
}

AssertCapabilityAttr *AssertCapabilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) AssertCapabilityAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AssertCapabilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((assert_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::assert_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " __attribute__((assert_shared_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 3 : {
    OS << " [[clang::assert_shared_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
}
}

const char *AssertCapabilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "assert_capability";
  case 1:
    return "assert_capability";
  case 2:
    return "assert_shared_capability";
  case 3:
    return "assert_shared_capability";
  }
}

AssertExclusiveLockAttr *AssertExclusiveLockAttr::clone(ASTContext &C) const {
  auto *A = new (C) AssertExclusiveLockAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AssertExclusiveLockAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((assert_exclusive_lock(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
}
}

const char *AssertExclusiveLockAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "assert_exclusive_lock";
  }
}

AssertSharedLockAttr *AssertSharedLockAttr::clone(ASTContext &C) const {
  auto *A = new (C) AssertSharedLockAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AssertSharedLockAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((assert_shared_lock(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
}
}

const char *AssertSharedLockAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "assert_shared_lock";
  }
}

AssumeAlignedAttr *AssumeAlignedAttr::clone(ASTContext &C) const {
  auto *A = new (C) AssumeAlignedAttr(C, *this, alignment, offset);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AssumeAlignedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((assume_aligned(" << getAlignment() << ", " << getOffset() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::assume_aligned(" << getAlignment() << ", " << getOffset() << ")]]";
    break;
  }
}
}

const char *AssumeAlignedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "assume_aligned";
  case 1:
    return "assume_aligned";
  }
}

AvailabilityAttr *AvailabilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) AvailabilityAttr(C, *this, platform, getIntroduced(), getDeprecated(), getObsoleted(), unavailable, getMessage(), strict, getReplacement(), priority);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void AvailabilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((availability(" << getPlatform()->getName();
  if (getStrict()) OS << ", strict";
  if (!getIntroduced().empty()) OS << ", introduced=" << getIntroduced();
  if (!getDeprecated().empty()) OS << ", deprecated=" << getDeprecated();
  if (!getObsoleted().empty()) OS << ", obsoleted=" << getObsoleted();
  if (getUnavailable()) OS << ", unavailable";
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::availability(" << (getPlatform() ? getPlatform()->getName() : "") << ", introduced=" << getIntroduced() << ", deprecated=" << getDeprecated() << ", obsoleted=" << getObsoleted() << ", " << getUnavailable() << ", \"" << getMessage() << "\", " << getStrict() << ", \"" << getReplacement() << "\", " << getPriority() << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::availability(" << (getPlatform() ? getPlatform()->getName() : "") << ", introduced=" << getIntroduced() << ", deprecated=" << getDeprecated() << ", obsoleted=" << getObsoleted() << ", " << getUnavailable() << ", \"" << getMessage() << "\", " << getStrict() << ", \"" << getReplacement() << "\", " << getPriority() << ")]]";
    break;
  }
}
}

const char *AvailabilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "availability";
  case 1:
    return "availability";
  case 2:
    return "availability";
  }
}

BPFPreserveAccessIndexAttr *BPFPreserveAccessIndexAttr::clone(ASTContext &C) const {
  auto *A = new (C) BPFPreserveAccessIndexAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void BPFPreserveAccessIndexAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((preserve_access_index))";
    break;
  }
  case 1 : {
    OS << " [[clang::preserve_access_index]]";
    break;
  }
  case 2 : {
    OS << " [[clang::preserve_access_index]]";
    break;
  }
}
}

const char *BPFPreserveAccessIndexAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "preserve_access_index";
  case 1:
    return "preserve_access_index";
  case 2:
    return "preserve_access_index";
  }
}

BlocksAttr *BlocksAttr::clone(ASTContext &C) const {
  auto *A = new (C) BlocksAttr(C, *this, type);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void BlocksAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((blocks(\"" << BlocksAttr::ConvertBlockTypeToStr(getType()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::blocks(\"" << BlocksAttr::ConvertBlockTypeToStr(getType()) << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::blocks(\"" << BlocksAttr::ConvertBlockTypeToStr(getType()) << "\")]]";
    break;
  }
}
}

const char *BlocksAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "blocks";
  case 1:
    return "blocks";
  case 2:
    return "blocks";
  }
}

C11NoReturnAttr *C11NoReturnAttr::clone(ASTContext &C) const {
  auto *A = new (C) C11NoReturnAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void C11NoReturnAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " _Noreturn";
    break;
  }
}
}

const char *C11NoReturnAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "_Noreturn";
  }
}

CDeclAttr *CDeclAttr::clone(ASTContext &C) const {
  auto *A = new (C) CDeclAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CDeclAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cdecl))";
    break;
  }
  case 1 : {
    OS << " [[gnu::cdecl]]";
    break;
  }
  case 2 : {
    OS << " __cdecl";
    break;
  }
  case 3 : {
    OS << " _cdecl";
    break;
  }
}
}

const char *CDeclAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cdecl";
  case 1:
    return "cdecl";
  case 2:
    return "__cdecl";
  case 3:
    return "_cdecl";
  }
}

CFAuditedTransferAttr *CFAuditedTransferAttr::clone(ASTContext &C) const {
  auto *A = new (C) CFAuditedTransferAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CFAuditedTransferAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cf_audited_transfer))";
    break;
  }
  case 1 : {
    OS << " [[clang::cf_audited_transfer]]";
    break;
  }
  case 2 : {
    OS << " [[clang::cf_audited_transfer]]";
    break;
  }
}
}

const char *CFAuditedTransferAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cf_audited_transfer";
  case 1:
    return "cf_audited_transfer";
  case 2:
    return "cf_audited_transfer";
  }
}

CFConsumedAttr *CFConsumedAttr::clone(ASTContext &C) const {
  auto *A = new (C) CFConsumedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CFConsumedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cf_consumed))";
    break;
  }
  case 1 : {
    OS << " [[clang::cf_consumed]]";
    break;
  }
  case 2 : {
    OS << " [[clang::cf_consumed]]";
    break;
  }
}
}

const char *CFConsumedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cf_consumed";
  case 1:
    return "cf_consumed";
  case 2:
    return "cf_consumed";
  }
}

CFGuardAttr *CFGuardAttr::clone(ASTContext &C) const {
  auto *A = new (C) CFGuardAttr(C, *this, guard);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CFGuardAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(guard(\"" << CFGuardAttr::ConvertGuardArgToStr(getGuard()) << "\"))";
    break;
  }
}
}

const char *CFGuardAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "guard";
  }
}

CFICanonicalJumpTableAttr *CFICanonicalJumpTableAttr::clone(ASTContext &C) const {
  auto *A = new (C) CFICanonicalJumpTableAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CFICanonicalJumpTableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cfi_canonical_jump_table))";
    break;
  }
  case 1 : {
    OS << " [[clang::cfi_canonical_jump_table]]";
    break;
  }
  case 2 : {
    OS << " [[clang::cfi_canonical_jump_table]]";
    break;
  }
}
}

const char *CFICanonicalJumpTableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cfi_canonical_jump_table";
  case 1:
    return "cfi_canonical_jump_table";
  case 2:
    return "cfi_canonical_jump_table";
  }
}

CFReturnsNotRetainedAttr *CFReturnsNotRetainedAttr::clone(ASTContext &C) const {
  auto *A = new (C) CFReturnsNotRetainedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CFReturnsNotRetainedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cf_returns_not_retained))";
    break;
  }
  case 1 : {
    OS << " [[clang::cf_returns_not_retained]]";
    break;
  }
  case 2 : {
    OS << " [[clang::cf_returns_not_retained]]";
    break;
  }
}
}

const char *CFReturnsNotRetainedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cf_returns_not_retained";
  case 1:
    return "cf_returns_not_retained";
  case 2:
    return "cf_returns_not_retained";
  }
}

CFReturnsRetainedAttr *CFReturnsRetainedAttr::clone(ASTContext &C) const {
  auto *A = new (C) CFReturnsRetainedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CFReturnsRetainedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cf_returns_retained))";
    break;
  }
  case 1 : {
    OS << " [[clang::cf_returns_retained]]";
    break;
  }
  case 2 : {
    OS << " [[clang::cf_returns_retained]]";
    break;
  }
}
}

const char *CFReturnsRetainedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cf_returns_retained";
  case 1:
    return "cf_returns_retained";
  case 2:
    return "cf_returns_retained";
  }
}

CFUnknownTransferAttr *CFUnknownTransferAttr::clone(ASTContext &C) const {
  auto *A = new (C) CFUnknownTransferAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CFUnknownTransferAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cf_unknown_transfer))";
    break;
  }
  case 1 : {
    OS << " [[clang::cf_unknown_transfer]]";
    break;
  }
  case 2 : {
    OS << " [[clang::cf_unknown_transfer]]";
    break;
  }
}
}

const char *CFUnknownTransferAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cf_unknown_transfer";
  case 1:
    return "cf_unknown_transfer";
  case 2:
    return "cf_unknown_transfer";
  }
}

CPUDispatchAttr *CPUDispatchAttr::clone(ASTContext &C) const {
  auto *A = new (C) CPUDispatchAttr(C, *this, cpus_, cpus_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CPUDispatchAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cpu_dispatch(";
  bool isFirst = true;
  for (const auto &Val : cpus()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::cpu_dispatch(";
  bool isFirst = true;
  for (const auto &Val : cpus()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::cpu_dispatch(";
  bool isFirst = true;
  for (const auto &Val : cpus()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 3 : {
    OS << " __declspec(cpu_dispatch(";
  bool isFirst = true;
  for (const auto &Val : cpus()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << "))";
    break;
  }
}
}

const char *CPUDispatchAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cpu_dispatch";
  case 1:
    return "cpu_dispatch";
  case 2:
    return "cpu_dispatch";
  case 3:
    return "cpu_dispatch";
  }
}

CPUSpecificAttr *CPUSpecificAttr::clone(ASTContext &C) const {
  auto *A = new (C) CPUSpecificAttr(C, *this, cpus_, cpus_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CPUSpecificAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cpu_specific(";
  bool isFirst = true;
  for (const auto &Val : cpus()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::cpu_specific(";
  bool isFirst = true;
  for (const auto &Val : cpus()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::cpu_specific(";
  bool isFirst = true;
  for (const auto &Val : cpus()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 3 : {
    OS << " __declspec(cpu_specific(";
  bool isFirst = true;
  for (const auto &Val : cpus()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << "))";
    break;
  }
}
}

const char *CPUSpecificAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cpu_specific";
  case 1:
    return "cpu_specific";
  case 2:
    return "cpu_specific";
  case 3:
    return "cpu_specific";
  }
}

CUDAConstantAttr *CUDAConstantAttr::clone(ASTContext &C) const {
  auto *A = new (C) CUDAConstantAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CUDAConstantAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((constant))";
    break;
  }
  case 1 : {
    OS << " __declspec(__constant__)";
    break;
  }
}
}

const char *CUDAConstantAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "constant";
  case 1:
    return "__constant__";
  }
}

CUDADeviceAttr *CUDADeviceAttr::clone(ASTContext &C) const {
  auto *A = new (C) CUDADeviceAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CUDADeviceAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((device))";
    break;
  }
  case 1 : {
    OS << " __declspec(__device__)";
    break;
  }
}
}

const char *CUDADeviceAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "device";
  case 1:
    return "__device__";
  }
}

CUDAGlobalAttr *CUDAGlobalAttr::clone(ASTContext &C) const {
  auto *A = new (C) CUDAGlobalAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CUDAGlobalAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((global))";
    break;
  }
  case 1 : {
    OS << " __declspec(__global__)";
    break;
  }
}
}

const char *CUDAGlobalAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "global";
  case 1:
    return "__global__";
  }
}

CUDAHostAttr *CUDAHostAttr::clone(ASTContext &C) const {
  auto *A = new (C) CUDAHostAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CUDAHostAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((host))";
    break;
  }
  case 1 : {
    OS << " __declspec(__host__)";
    break;
  }
}
}

const char *CUDAHostAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "host";
  case 1:
    return "__host__";
  }
}

CUDAInvalidTargetAttr *CUDAInvalidTargetAttr::clone(ASTContext &C) const {
  auto *A = new (C) CUDAInvalidTargetAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CUDAInvalidTargetAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *CUDAInvalidTargetAttr::getSpelling() const {
  return "(No spelling)";
}

CUDALaunchBoundsAttr *CUDALaunchBoundsAttr::clone(ASTContext &C) const {
  auto *A = new (C) CUDALaunchBoundsAttr(C, *this, maxThreads, minBlocks);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CUDALaunchBoundsAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((launch_bounds(" << getMaxThreads() << ", " << getMinBlocks() << ")))";
    break;
  }
  case 1 : {
    OS << " __declspec(__launch_bounds__(" << getMaxThreads() << ", " << getMinBlocks() << "))";
    break;
  }
}
}

const char *CUDALaunchBoundsAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "launch_bounds";
  case 1:
    return "__launch_bounds__";
  }
}

CUDASharedAttr *CUDASharedAttr::clone(ASTContext &C) const {
  auto *A = new (C) CUDASharedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CUDASharedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((shared))";
    break;
  }
  case 1 : {
    OS << " __declspec(__shared__)";
    break;
  }
}
}

const char *CUDASharedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "shared";
  case 1:
    return "__shared__";
  }
}

CXX11NoReturnAttr *CXX11NoReturnAttr::clone(ASTContext &C) const {
  auto *A = new (C) CXX11NoReturnAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CXX11NoReturnAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " [[noreturn]]";
    break;
  }
}
}

const char *CXX11NoReturnAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "noreturn";
  }
}

CallableWhenAttr *CallableWhenAttr::clone(ASTContext &C) const {
  auto *A = new (C) CallableWhenAttr(C, *this, callableStates_, callableStates_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CallableWhenAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((callable_when(";
  bool isFirst = true;
  for (const auto &Val : callableStates()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << CallableWhenAttr::ConvertConsumedStateToStr(Val)<< "\"";
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::callable_when(";
  bool isFirst = true;
  for (const auto &Val : callableStates()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << CallableWhenAttr::ConvertConsumedStateToStr(Val)<< "\"";
  }
  OS << ")]]";
    break;
  }
}
}

const char *CallableWhenAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "callable_when";
  case 1:
    return "callable_when";
  }
}

CallbackAttr *CallbackAttr::clone(ASTContext &C) const {
  auto *A = new (C) CallbackAttr(C, *this, encoding_, encoding_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CallbackAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((callback(";
  bool isFirst = true;
  for (const auto &Val : encoding()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::callback(";
  bool isFirst = true;
  for (const auto &Val : encoding()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::callback(";
  bool isFirst = true;
  for (const auto &Val : encoding()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
}
}

const char *CallbackAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "callback";
  case 1:
    return "callback";
  case 2:
    return "callback";
  }
}

CapabilityAttr *CapabilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) CapabilityAttr(C, *this, getName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CapabilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((capability(\"" << getName() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::capability(\"" << getName() << "\")]]";
    break;
  }
  case 2 : {
    OS << " __attribute__((shared_capability(\"" << getName() << "\")))";
    break;
  }
  case 3 : {
    OS << " [[clang::shared_capability(\"" << getName() << "\")]]";
    break;
  }
}
}

const char *CapabilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "capability";
  case 1:
    return "capability";
  case 2:
    return "shared_capability";
  case 3:
    return "shared_capability";
  }
}

CapturedRecordAttr *CapturedRecordAttr::clone(ASTContext &C) const {
  auto *A = new (C) CapturedRecordAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CapturedRecordAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *CapturedRecordAttr::getSpelling() const {
  return "(No spelling)";
}

CarriesDependencyAttr *CarriesDependencyAttr::clone(ASTContext &C) const {
  auto *A = new (C) CarriesDependencyAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CarriesDependencyAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((carries_dependency))";
    break;
  }
  case 1 : {
    OS << " [[carries_dependency]]";
    break;
  }
}
}

const char *CarriesDependencyAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "carries_dependency";
  case 1:
    return "carries_dependency";
  }
}

CleanupAttr *CleanupAttr::clone(ASTContext &C) const {
  auto *A = new (C) CleanupAttr(C, *this, functionDecl);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CleanupAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cleanup(" << getFunctionDecl()->getNameInfo().getAsString() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::cleanup(" << getFunctionDecl()->getNameInfo().getAsString() << ")]]";
    break;
  }
}
}

const char *CleanupAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cleanup";
  case 1:
    return "cleanup";
  }
}

CodeSegAttr *CodeSegAttr::clone(ASTContext &C) const {
  auto *A = new (C) CodeSegAttr(C, *this, getName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CodeSegAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(code_seg(\"" << getName() << "\"))";
    break;
  }
}
}

const char *CodeSegAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "code_seg";
  }
}

ColdAttr *ColdAttr::clone(ASTContext &C) const {
  auto *A = new (C) ColdAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ColdAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((cold))";
    break;
  }
  case 1 : {
    OS << " [[gnu::cold]]";
    break;
  }
}
}

const char *ColdAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "cold";
  case 1:
    return "cold";
  }
}

CommonAttr *CommonAttr::clone(ASTContext &C) const {
  auto *A = new (C) CommonAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void CommonAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((common))";
    break;
  }
  case 1 : {
    OS << " [[gnu::common]]";
    break;
  }
}
}

const char *CommonAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "common";
  case 1:
    return "common";
  }
}

ConstAttr *ConstAttr::clone(ASTContext &C) const {
  auto *A = new (C) ConstAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ConstAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((const))";
    break;
  }
  case 1 : {
    OS << " [[gnu::const]]";
    break;
  }
  case 2 : {
    OS << " __attribute__((__const))";
    break;
  }
  case 3 : {
    OS << " [[gnu::__const]]";
    break;
  }
}
}

const char *ConstAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "const";
  case 1:
    return "const";
  case 2:
    return "__const";
  case 3:
    return "__const";
  }
}

ConstInitAttr *ConstInitAttr::clone(ASTContext &C) const {
  auto *A = new (C) ConstInitAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ConstInitAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " constinit";
    break;
  }
  case 1 : {
    OS << " __attribute__((require_constant_initialization))";
    break;
  }
  case 2 : {
    OS << " [[clang::require_constant_initialization]]";
    break;
  }
}
}

const char *ConstInitAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "constinit";
  case 1:
    return "require_constant_initialization";
  case 2:
    return "require_constant_initialization";
  }
}

ConstructorAttr *ConstructorAttr::clone(ASTContext &C) const {
  auto *A = new (C) ConstructorAttr(C, *this, priority);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ConstructorAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((constructor(" << getPriority() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::constructor(" << getPriority() << ")]]";
    break;
  }
}
}

const char *ConstructorAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "constructor";
  case 1:
    return "constructor";
  }
}

ConsumableAttr *ConsumableAttr::clone(ASTContext &C) const {
  auto *A = new (C) ConsumableAttr(C, *this, defaultState);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ConsumableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((consumable(\"" << ConsumableAttr::ConvertConsumedStateToStr(getDefaultState()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::consumable(\"" << ConsumableAttr::ConvertConsumedStateToStr(getDefaultState()) << "\")]]";
    break;
  }
}
}

const char *ConsumableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "consumable";
  case 1:
    return "consumable";
  }
}

ConsumableAutoCastAttr *ConsumableAutoCastAttr::clone(ASTContext &C) const {
  auto *A = new (C) ConsumableAutoCastAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ConsumableAutoCastAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((consumable_auto_cast_state))";
    break;
  }
  case 1 : {
    OS << " [[clang::consumable_auto_cast_state]]";
    break;
  }
}
}

const char *ConsumableAutoCastAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "consumable_auto_cast_state";
  case 1:
    return "consumable_auto_cast_state";
  }
}

ConsumableSetOnReadAttr *ConsumableSetOnReadAttr::clone(ASTContext &C) const {
  auto *A = new (C) ConsumableSetOnReadAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ConsumableSetOnReadAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((consumable_set_state_on_read))";
    break;
  }
  case 1 : {
    OS << " [[clang::consumable_set_state_on_read]]";
    break;
  }
}
}

const char *ConsumableSetOnReadAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "consumable_set_state_on_read";
  case 1:
    return "consumable_set_state_on_read";
  }
}

ConvergentAttr *ConvergentAttr::clone(ASTContext &C) const {
  auto *A = new (C) ConvergentAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ConvergentAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((convergent))";
    break;
  }
  case 1 : {
    OS << " [[clang::convergent]]";
    break;
  }
  case 2 : {
    OS << " [[clang::convergent]]";
    break;
  }
}
}

const char *ConvergentAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "convergent";
  case 1:
    return "convergent";
  case 2:
    return "convergent";
  }
}

DLLExportAttr *DLLExportAttr::clone(ASTContext &C) const {
  auto *A = new (C) DLLExportAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void DLLExportAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(dllexport)";
    break;
  }
  case 1 : {
    OS << " __attribute__((dllexport))";
    break;
  }
  case 2 : {
    OS << " [[gnu::dllexport]]";
    break;
  }
}
}

const char *DLLExportAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "dllexport";
  case 1:
    return "dllexport";
  case 2:
    return "dllexport";
  }
}

DLLExportStaticLocalAttr *DLLExportStaticLocalAttr::clone(ASTContext &C) const {
  auto *A = new (C) DLLExportStaticLocalAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void DLLExportStaticLocalAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *DLLExportStaticLocalAttr::getSpelling() const {
  return "(No spelling)";
}

DLLImportAttr *DLLImportAttr::clone(ASTContext &C) const {
  auto *A = new (C) DLLImportAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void DLLImportAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(dllimport)";
    break;
  }
  case 1 : {
    OS << " __attribute__((dllimport))";
    break;
  }
  case 2 : {
    OS << " [[gnu::dllimport]]";
    break;
  }
}
}

const char *DLLImportAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "dllimport";
  case 1:
    return "dllimport";
  case 2:
    return "dllimport";
  }
}

DLLImportStaticLocalAttr *DLLImportStaticLocalAttr::clone(ASTContext &C) const {
  auto *A = new (C) DLLImportStaticLocalAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void DLLImportStaticLocalAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *DLLImportStaticLocalAttr::getSpelling() const {
  return "(No spelling)";
}

DeprecatedAttr *DeprecatedAttr::clone(ASTContext &C) const {
  auto *A = new (C) DeprecatedAttr(C, *this, getMessage(), getReplacement());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void DeprecatedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((deprecated(\"" << getMessage() << "\"";
    if (!getReplacement().empty()) OS << ", \"" << getReplacement() << "\"";
    OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::deprecated(\"" << getMessage() << "\"";
    OS << ")]]";
    break;
  }
  case 2 : {
    OS << " __declspec(deprecated(\"" << getMessage() << "\"";
    OS << "))";
    break;
  }
  case 3 : {
    OS << " [[deprecated(\"" << getMessage() << "\"";
    OS << ")]]";
    break;
  }
  case 4 : {
    OS << " [[deprecated(\"" << getMessage() << "\"";
    OS << ")]]";
    break;
  }
}
}

const char *DeprecatedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "deprecated";
  case 1:
    return "deprecated";
  case 2:
    return "deprecated";
  case 3:
    return "deprecated";
  case 4:
    return "deprecated";
  }
}

DestructorAttr *DestructorAttr::clone(ASTContext &C) const {
  auto *A = new (C) DestructorAttr(C, *this, priority);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void DestructorAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((destructor(" << getPriority() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::destructor(" << getPriority() << ")]]";
    break;
  }
}
}

const char *DestructorAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "destructor";
  case 1:
    return "destructor";
  }
}

DiagnoseIfAttr *DiagnoseIfAttr::clone(ASTContext &C) const {
  auto *A = new (C) DiagnoseIfAttr(C, *this, cond, getMessage(), diagnosticType, argDependent, parent);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void DiagnoseIfAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((diagnose_if(" << getCond() << ", \"" << getMessage() << "\", \"" << DiagnoseIfAttr::ConvertDiagnosticTypeToStr(getDiagnosticType()) << "\")))";
    break;
  }
}
}

const char *DiagnoseIfAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "diagnose_if";
  }
}

DisableTailCallsAttr *DisableTailCallsAttr::clone(ASTContext &C) const {
  auto *A = new (C) DisableTailCallsAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void DisableTailCallsAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((disable_tail_calls))";
    break;
  }
  case 1 : {
    OS << " [[clang::disable_tail_calls]]";
    break;
  }
  case 2 : {
    OS << " [[clang::disable_tail_calls]]";
    break;
  }
}
}

const char *DisableTailCallsAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "disable_tail_calls";
  case 1:
    return "disable_tail_calls";
  case 2:
    return "disable_tail_calls";
  }
}

EmptyBasesAttr *EmptyBasesAttr::clone(ASTContext &C) const {
  auto *A = new (C) EmptyBasesAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void EmptyBasesAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(empty_bases)";
    break;
  }
}
}

const char *EmptyBasesAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "empty_bases";
  }
}

EnableIfAttr *EnableIfAttr::clone(ASTContext &C) const {
  auto *A = new (C) EnableIfAttr(C, *this, cond, getMessage());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void EnableIfAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((enable_if(" << getCond() << ", \"" << getMessage() << "\")))";
    break;
  }
}
}

const char *EnableIfAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "enable_if";
  }
}

EnumExtensibilityAttr *EnumExtensibilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) EnumExtensibilityAttr(C, *this, extensibility);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void EnumExtensibilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((enum_extensibility(\"" << EnumExtensibilityAttr::ConvertKindToStr(getExtensibility()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::enum_extensibility(\"" << EnumExtensibilityAttr::ConvertKindToStr(getExtensibility()) << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::enum_extensibility(\"" << EnumExtensibilityAttr::ConvertKindToStr(getExtensibility()) << "\")]]";
    break;
  }
}
}

const char *EnumExtensibilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "enum_extensibility";
  case 1:
    return "enum_extensibility";
  case 2:
    return "enum_extensibility";
  }
}

ExcludeFromExplicitInstantiationAttr *ExcludeFromExplicitInstantiationAttr::clone(ASTContext &C) const {
  auto *A = new (C) ExcludeFromExplicitInstantiationAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ExcludeFromExplicitInstantiationAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((exclude_from_explicit_instantiation))";
    break;
  }
  case 1 : {
    OS << " [[clang::exclude_from_explicit_instantiation]]";
    break;
  }
  case 2 : {
    OS << " [[clang::exclude_from_explicit_instantiation]]";
    break;
  }
}
}

const char *ExcludeFromExplicitInstantiationAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "exclude_from_explicit_instantiation";
  case 1:
    return "exclude_from_explicit_instantiation";
  case 2:
    return "exclude_from_explicit_instantiation";
  }
}

ExclusiveTrylockFunctionAttr *ExclusiveTrylockFunctionAttr::clone(ASTContext &C) const {
  auto *A = new (C) ExclusiveTrylockFunctionAttr(C, *this, successValue, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ExclusiveTrylockFunctionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((exclusive_trylock_function(" << getSuccessValue() << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
}
}

const char *ExclusiveTrylockFunctionAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "exclusive_trylock_function";
  }
}

ExternalSourceSymbolAttr *ExternalSourceSymbolAttr::clone(ASTContext &C) const {
  auto *A = new (C) ExternalSourceSymbolAttr(C, *this, getLanguage(), getDefinedIn(), generatedDeclaration);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ExternalSourceSymbolAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((external_source_symbol(\"" << getLanguage() << "\", \"" << getDefinedIn() << "\", " << getGeneratedDeclaration() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::external_source_symbol(\"" << getLanguage() << "\", \"" << getDefinedIn() << "\", " << getGeneratedDeclaration() << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::external_source_symbol(\"" << getLanguage() << "\", \"" << getDefinedIn() << "\", " << getGeneratedDeclaration() << ")]]";
    break;
  }
}
}

const char *ExternalSourceSymbolAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "external_source_symbol";
  case 1:
    return "external_source_symbol";
  case 2:
    return "external_source_symbol";
  }
}

FallThroughAttr *FallThroughAttr::clone(ASTContext &C) const {
  auto *A = new (C) FallThroughAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void FallThroughAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " [[fallthrough]]";
    break;
  }
  case 1 : {
    OS << " [[fallthrough]]";
    break;
  }
  case 2 : {
    OS << " [[clang::fallthrough]]";
    break;
  }
  case 3 : {
    OS << " __attribute__((fallthrough))";
    break;
  }
  case 4 : {
    OS << " [[gnu::fallthrough]]";
    break;
  }
}
}

const char *FallThroughAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "fallthrough";
  case 1:
    return "fallthrough";
  case 2:
    return "fallthrough";
  case 3:
    return "fallthrough";
  case 4:
    return "fallthrough";
  }
}

FastCallAttr *FastCallAttr::clone(ASTContext &C) const {
  auto *A = new (C) FastCallAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void FastCallAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((fastcall))";
    break;
  }
  case 1 : {
    OS << " [[gnu::fastcall]]";
    break;
  }
  case 2 : {
    OS << " __fastcall";
    break;
  }
  case 3 : {
    OS << " _fastcall";
    break;
  }
}
}

const char *FastCallAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "fastcall";
  case 1:
    return "fastcall";
  case 2:
    return "__fastcall";
  case 3:
    return "_fastcall";
  }
}

FinalAttr *FinalAttr::clone(ASTContext &C) const {
  auto *A = new (C) FinalAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void FinalAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " final";
    break;
  }
  case 1 : {
    OS << " sealed";
    break;
  }
}
}

const char *FinalAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "final";
  case 1:
    return "sealed";
  }
}

FlagEnumAttr *FlagEnumAttr::clone(ASTContext &C) const {
  auto *A = new (C) FlagEnumAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void FlagEnumAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((flag_enum))";
    break;
  }
  case 1 : {
    OS << " [[clang::flag_enum]]";
    break;
  }
  case 2 : {
    OS << " [[clang::flag_enum]]";
    break;
  }
}
}

const char *FlagEnumAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "flag_enum";
  case 1:
    return "flag_enum";
  case 2:
    return "flag_enum";
  }
}

FlattenAttr *FlattenAttr::clone(ASTContext &C) const {
  auto *A = new (C) FlattenAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void FlattenAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((flatten))";
    break;
  }
  case 1 : {
    OS << " [[gnu::flatten]]";
    break;
  }
}
}

const char *FlattenAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "flatten";
  case 1:
    return "flatten";
  }
}

FormatAttr *FormatAttr::clone(ASTContext &C) const {
  auto *A = new (C) FormatAttr(C, *this, type, formatIdx, firstArg);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void FormatAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((format(" << (getType() ? getType()->getName() : "") << ", " << getFormatIdx() << ", " << getFirstArg() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::format(" << (getType() ? getType()->getName() : "") << ", " << getFormatIdx() << ", " << getFirstArg() << ")]]";
    break;
  }
}
}

const char *FormatAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "format";
  case 1:
    return "format";
  }
}

FormatArgAttr *FormatArgAttr::clone(ASTContext &C) const {
  auto *A = new (C) FormatArgAttr(C, *this, formatIdx);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void FormatArgAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((format_arg(" << getFormatIdx().getSourceIndex() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::format_arg(" << getFormatIdx().getSourceIndex() << ")]]";
    break;
  }
}
}

const char *FormatArgAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "format_arg";
  case 1:
    return "format_arg";
  }
}

GNUInlineAttr *GNUInlineAttr::clone(ASTContext &C) const {
  auto *A = new (C) GNUInlineAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void GNUInlineAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((gnu_inline))";
    break;
  }
  case 1 : {
    OS << " [[gnu::gnu_inline]]";
    break;
  }
}
}

const char *GNUInlineAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "gnu_inline";
  case 1:
    return "gnu_inline";
  }
}

GuardedByAttr *GuardedByAttr::clone(ASTContext &C) const {
  auto *A = new (C) GuardedByAttr(C, *this, arg);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void GuardedByAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((guarded_by(" << getArg() << ")))";
    break;
  }
}
}

const char *GuardedByAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "guarded_by";
  }
}

GuardedVarAttr *GuardedVarAttr::clone(ASTContext &C) const {
  auto *A = new (C) GuardedVarAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void GuardedVarAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((guarded_var))";
    break;
  }
  case 1 : {
    OS << " [[clang::guarded_var]]";
    break;
  }
}
}

const char *GuardedVarAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "guarded_var";
  case 1:
    return "guarded_var";
  }
}

HIPPinnedShadowAttr *HIPPinnedShadowAttr::clone(ASTContext &C) const {
  auto *A = new (C) HIPPinnedShadowAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void HIPPinnedShadowAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((hip_pinned_shadow))";
    break;
  }
  case 1 : {
    OS << " __declspec(__hip_pinned_shadow__)";
    break;
  }
}
}

const char *HIPPinnedShadowAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "hip_pinned_shadow";
  case 1:
    return "__hip_pinned_shadow__";
  }
}

HotAttr *HotAttr::clone(ASTContext &C) const {
  auto *A = new (C) HotAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void HotAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((hot))";
    break;
  }
  case 1 : {
    OS << " [[gnu::hot]]";
    break;
  }
}
}

const char *HotAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "hot";
  case 1:
    return "hot";
  }
}

IBActionAttr *IBActionAttr::clone(ASTContext &C) const {
  auto *A = new (C) IBActionAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void IBActionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ibaction))";
    break;
  }
  case 1 : {
    OS << " [[clang::ibaction]]";
    break;
  }
  case 2 : {
    OS << " [[clang::ibaction]]";
    break;
  }
}
}

const char *IBActionAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ibaction";
  case 1:
    return "ibaction";
  case 2:
    return "ibaction";
  }
}

IBOutletAttr *IBOutletAttr::clone(ASTContext &C) const {
  auto *A = new (C) IBOutletAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void IBOutletAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((iboutlet))";
    break;
  }
  case 1 : {
    OS << " [[clang::iboutlet]]";
    break;
  }
  case 2 : {
    OS << " [[clang::iboutlet]]";
    break;
  }
}
}

const char *IBOutletAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "iboutlet";
  case 1:
    return "iboutlet";
  case 2:
    return "iboutlet";
  }
}

IBOutletCollectionAttr *IBOutletCollectionAttr::clone(ASTContext &C) const {
  auto *A = new (C) IBOutletCollectionAttr(C, *this, interface_);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void IBOutletCollectionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((iboutletcollection";
    unsigned TrailingOmittedArgs = 0;
    if (!getInterfaceLoc())
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!getInterfaceLoc())) {
      OS << "" << getInterface().getAsString() << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << "))";
    break;
  }
  case 1 : {
    OS << " [[clang::iboutletcollection";
    unsigned TrailingOmittedArgs = 0;
    if (!getInterfaceLoc())
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!getInterfaceLoc())) {
      OS << "" << getInterface().getAsString() << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << "]]";
    break;
  }
  case 2 : {
    OS << " [[clang::iboutletcollection";
    unsigned TrailingOmittedArgs = 0;
    if (!getInterfaceLoc())
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!getInterfaceLoc())) {
      OS << "" << getInterface().getAsString() << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << "]]";
    break;
  }
}
}

const char *IBOutletCollectionAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "iboutletcollection";
  case 1:
    return "iboutletcollection";
  case 2:
    return "iboutletcollection";
  }
}

IFuncAttr *IFuncAttr::clone(ASTContext &C) const {
  auto *A = new (C) IFuncAttr(C, *this, getResolver());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void IFuncAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ifunc(\"" << getResolver() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::ifunc(\"" << getResolver() << "\")]]";
    break;
  }
}
}

const char *IFuncAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ifunc";
  case 1:
    return "ifunc";
  }
}

InitPriorityAttr *InitPriorityAttr::clone(ASTContext &C) const {
  auto *A = new (C) InitPriorityAttr(C, *this, priority);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void InitPriorityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((init_priority(" << getPriority() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::init_priority(" << getPriority() << ")]]";
    break;
  }
}
}

const char *InitPriorityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "init_priority";
  case 1:
    return "init_priority";
  }
}

InitSegAttr *InitSegAttr::clone(ASTContext &C) const {
  auto *A = new (C) InitSegAttr(C, *this, getSection());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void InitSegAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << "#pragma init_seg";
    printPrettyPragma(OS, Policy);
    OS << "\n";    break;
  }
}
}

const char *InitSegAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "init_seg";
  }
}

IntelOclBiccAttr *IntelOclBiccAttr::clone(ASTContext &C) const {
  auto *A = new (C) IntelOclBiccAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void IntelOclBiccAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((intel_ocl_bicc))";
    break;
  }
  case 1 : {
    OS << " [[clang::intel_ocl_bicc]]";
    break;
  }
}
}

const char *IntelOclBiccAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "intel_ocl_bicc";
  case 1:
    return "intel_ocl_bicc";
  }
}

InternalLinkageAttr *InternalLinkageAttr::clone(ASTContext &C) const {
  auto *A = new (C) InternalLinkageAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void InternalLinkageAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((internal_linkage))";
    break;
  }
  case 1 : {
    OS << " [[clang::internal_linkage]]";
    break;
  }
  case 2 : {
    OS << " [[clang::internal_linkage]]";
    break;
  }
}
}

const char *InternalLinkageAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "internal_linkage";
  case 1:
    return "internal_linkage";
  case 2:
    return "internal_linkage";
  }
}

LTOVisibilityPublicAttr *LTOVisibilityPublicAttr::clone(ASTContext &C) const {
  auto *A = new (C) LTOVisibilityPublicAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void LTOVisibilityPublicAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((lto_visibility_public))";
    break;
  }
  case 1 : {
    OS << " [[clang::lto_visibility_public]]";
    break;
  }
  case 2 : {
    OS << " [[clang::lto_visibility_public]]";
    break;
  }
}
}

const char *LTOVisibilityPublicAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "lto_visibility_public";
  case 1:
    return "lto_visibility_public";
  case 2:
    return "lto_visibility_public";
  }
}

LayoutVersionAttr *LayoutVersionAttr::clone(ASTContext &C) const {
  auto *A = new (C) LayoutVersionAttr(C, *this, version);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void LayoutVersionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(layout_version(" << getVersion() << "))";
    break;
  }
}
}

const char *LayoutVersionAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "layout_version";
  }
}

LifetimeBoundAttr *LifetimeBoundAttr::clone(ASTContext &C) const {
  auto *A = new (C) LifetimeBoundAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void LifetimeBoundAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((lifetimebound))";
    break;
  }
  case 1 : {
    OS << " [[clang::lifetimebound]]";
    break;
  }
}
}

const char *LifetimeBoundAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "lifetimebound";
  case 1:
    return "lifetimebound";
  }
}

LockReturnedAttr *LockReturnedAttr::clone(ASTContext &C) const {
  auto *A = new (C) LockReturnedAttr(C, *this, arg);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void LockReturnedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((lock_returned(" << getArg() << ")))";
    break;
  }
}
}

const char *LockReturnedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "lock_returned";
  }
}

LocksExcludedAttr *LocksExcludedAttr::clone(ASTContext &C) const {
  auto *A = new (C) LocksExcludedAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void LocksExcludedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((locks_excluded(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
}
}

const char *LocksExcludedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "locks_excluded";
  }
}

LoopHintAttr *LoopHintAttr::clone(ASTContext &C) const {
  auto *A = new (C) LoopHintAttr(C, *this, option, state, value);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void LoopHintAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << "#pragma clang loop";
    printPrettyPragma(OS, Policy);
    OS << "\n";    break;
  }
  case 1 : {
    OS << "#pragma unroll";
    printPrettyPragma(OS, Policy);
    OS << "\n";    break;
  }
  case 2 : {
    OS << "#pragma nounroll";
    printPrettyPragma(OS, Policy);
    OS << "\n";    break;
  }
  case 3 : {
    OS << "#pragma unroll_and_jam";
    printPrettyPragma(OS, Policy);
    OS << "\n";    break;
  }
  case 4 : {
    OS << "#pragma nounroll_and_jam";
    printPrettyPragma(OS, Policy);
    OS << "\n";    break;
  }
}
}

const char *LoopHintAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "loop";
  case 1:
    return "unroll";
  case 2:
    return "nounroll";
  case 3:
    return "unroll_and_jam";
  case 4:
    return "nounroll_and_jam";
  }
}

MIGServerRoutineAttr *MIGServerRoutineAttr::clone(ASTContext &C) const {
  auto *A = new (C) MIGServerRoutineAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MIGServerRoutineAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((mig_server_routine))";
    break;
  }
  case 1 : {
    OS << " [[clang::mig_server_routine]]";
    break;
  }
  case 2 : {
    OS << " [[clang::mig_server_routine]]";
    break;
  }
}
}

const char *MIGServerRoutineAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "mig_server_routine";
  case 1:
    return "mig_server_routine";
  case 2:
    return "mig_server_routine";
  }
}

MSABIAttr *MSABIAttr::clone(ASTContext &C) const {
  auto *A = new (C) MSABIAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MSABIAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ms_abi))";
    break;
  }
  case 1 : {
    OS << " [[gnu::ms_abi]]";
    break;
  }
}
}

const char *MSABIAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ms_abi";
  case 1:
    return "ms_abi";
  }
}

MSAllocatorAttr *MSAllocatorAttr::clone(ASTContext &C) const {
  auto *A = new (C) MSAllocatorAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MSAllocatorAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(allocator)";
    break;
  }
}
}

const char *MSAllocatorAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "allocator";
  }
}

MSInheritanceAttr *MSInheritanceAttr::clone(ASTContext &C) const {
  auto *A = new (C) MSInheritanceAttr(C, *this, bestCase);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MSInheritanceAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __single_inheritance";
    break;
  }
  case 1 : {
    OS << " __multiple_inheritance";
    break;
  }
  case 2 : {
    OS << " __virtual_inheritance";
    break;
  }
  case 3 : {
    OS << " __unspecified_inheritance";
    break;
  }
}
}

const char *MSInheritanceAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__single_inheritance";
  case 1:
    return "__multiple_inheritance";
  case 2:
    return "__virtual_inheritance";
  case 3:
    return "__unspecified_inheritance";
  }
}

MSNoVTableAttr *MSNoVTableAttr::clone(ASTContext &C) const {
  auto *A = new (C) MSNoVTableAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MSNoVTableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(novtable)";
    break;
  }
}
}

const char *MSNoVTableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "novtable";
  }
}

MSP430InterruptAttr *MSP430InterruptAttr::clone(ASTContext &C) const {
  auto *A = new (C) MSP430InterruptAttr(C, *this, number);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MSP430InterruptAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((interrupt(" << getNumber() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::interrupt(" << getNumber() << ")]]";
    break;
  }
}
}

const char *MSP430InterruptAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "interrupt";
  case 1:
    return "interrupt";
  }
}

MSStructAttr *MSStructAttr::clone(ASTContext &C) const {
  auto *A = new (C) MSStructAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MSStructAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ms_struct))";
    break;
  }
  case 1 : {
    OS << " [[gnu::ms_struct]]";
    break;
  }
}
}

const char *MSStructAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ms_struct";
  case 1:
    return "ms_struct";
  }
}

MSVtorDispAttr *MSVtorDispAttr::clone(ASTContext &C) const {
  auto *A = new (C) MSVtorDispAttr(C, *this, vdm);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MSVtorDispAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *MSVtorDispAttr::getSpelling() const {
  return "(No spelling)";
}

MaxFieldAlignmentAttr *MaxFieldAlignmentAttr::clone(ASTContext &C) const {
  auto *A = new (C) MaxFieldAlignmentAttr(C, *this, alignment);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MaxFieldAlignmentAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *MaxFieldAlignmentAttr::getSpelling() const {
  return "(No spelling)";
}

MayAliasAttr *MayAliasAttr::clone(ASTContext &C) const {
  auto *A = new (C) MayAliasAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MayAliasAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((may_alias))";
    break;
  }
  case 1 : {
    OS << " [[gnu::may_alias]]";
    break;
  }
}
}

const char *MayAliasAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "may_alias";
  case 1:
    return "may_alias";
  }
}

MicroMipsAttr *MicroMipsAttr::clone(ASTContext &C) const {
  auto *A = new (C) MicroMipsAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MicroMipsAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((micromips))";
    break;
  }
  case 1 : {
    OS << " [[gnu::micromips]]";
    break;
  }
}
}

const char *MicroMipsAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "micromips";
  case 1:
    return "micromips";
  }
}

MinSizeAttr *MinSizeAttr::clone(ASTContext &C) const {
  auto *A = new (C) MinSizeAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MinSizeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((minsize))";
    break;
  }
  case 1 : {
    OS << " [[clang::minsize]]";
    break;
  }
  case 2 : {
    OS << " [[clang::minsize]]";
    break;
  }
}
}

const char *MinSizeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "minsize";
  case 1:
    return "minsize";
  case 2:
    return "minsize";
  }
}

MinVectorWidthAttr *MinVectorWidthAttr::clone(ASTContext &C) const {
  auto *A = new (C) MinVectorWidthAttr(C, *this, vectorWidth);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MinVectorWidthAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((min_vector_width(" << getVectorWidth() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::min_vector_width(" << getVectorWidth() << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::min_vector_width(" << getVectorWidth() << ")]]";
    break;
  }
}
}

const char *MinVectorWidthAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "min_vector_width";
  case 1:
    return "min_vector_width";
  case 2:
    return "min_vector_width";
  }
}

Mips16Attr *Mips16Attr::clone(ASTContext &C) const {
  auto *A = new (C) Mips16Attr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void Mips16Attr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((mips16))";
    break;
  }
  case 1 : {
    OS << " [[gnu::mips16]]";
    break;
  }
}
}

const char *Mips16Attr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "mips16";
  case 1:
    return "mips16";
  }
}

MipsInterruptAttr *MipsInterruptAttr::clone(ASTContext &C) const {
  auto *A = new (C) MipsInterruptAttr(C, *this, interrupt);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MipsInterruptAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((interrupt(\"" << MipsInterruptAttr::ConvertInterruptTypeToStr(getInterrupt()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::interrupt(\"" << MipsInterruptAttr::ConvertInterruptTypeToStr(getInterrupt()) << "\")]]";
    break;
  }
}
}

const char *MipsInterruptAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "interrupt";
  case 1:
    return "interrupt";
  }
}

MipsLongCallAttr *MipsLongCallAttr::clone(ASTContext &C) const {
  auto *A = new (C) MipsLongCallAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MipsLongCallAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((long_call))";
    break;
  }
  case 1 : {
    OS << " [[gnu::long_call]]";
    break;
  }
  case 2 : {
    OS << " __attribute__((far))";
    break;
  }
  case 3 : {
    OS << " [[gnu::far]]";
    break;
  }
}
}

const char *MipsLongCallAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "long_call";
  case 1:
    return "long_call";
  case 2:
    return "far";
  case 3:
    return "far";
  }
}

MipsShortCallAttr *MipsShortCallAttr::clone(ASTContext &C) const {
  auto *A = new (C) MipsShortCallAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void MipsShortCallAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((short_call))";
    break;
  }
  case 1 : {
    OS << " [[gnu::short_call]]";
    break;
  }
  case 2 : {
    OS << " __attribute__((near))";
    break;
  }
  case 3 : {
    OS << " [[gnu::near]]";
    break;
  }
}
}

const char *MipsShortCallAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "short_call";
  case 1:
    return "short_call";
  case 2:
    return "near";
  case 3:
    return "near";
  }
}

ModeAttr *ModeAttr::clone(ASTContext &C) const {
  auto *A = new (C) ModeAttr(C, *this, mode);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ModeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((mode(" << (getMode() ? getMode()->getName() : "") << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::mode(" << (getMode() ? getMode()->getName() : "") << ")]]";
    break;
  }
}
}

const char *ModeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "mode";
  case 1:
    return "mode";
  }
}

NSConsumedAttr *NSConsumedAttr::clone(ASTContext &C) const {
  auto *A = new (C) NSConsumedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NSConsumedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ns_consumed))";
    break;
  }
  case 1 : {
    OS << " [[clang::ns_consumed]]";
    break;
  }
  case 2 : {
    OS << " [[clang::ns_consumed]]";
    break;
  }
}
}

const char *NSConsumedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ns_consumed";
  case 1:
    return "ns_consumed";
  case 2:
    return "ns_consumed";
  }
}

NSConsumesSelfAttr *NSConsumesSelfAttr::clone(ASTContext &C) const {
  auto *A = new (C) NSConsumesSelfAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NSConsumesSelfAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ns_consumes_self))";
    break;
  }
  case 1 : {
    OS << " [[clang::ns_consumes_self]]";
    break;
  }
  case 2 : {
    OS << " [[clang::ns_consumes_self]]";
    break;
  }
}
}

const char *NSConsumesSelfAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ns_consumes_self";
  case 1:
    return "ns_consumes_self";
  case 2:
    return "ns_consumes_self";
  }
}

NSReturnsAutoreleasedAttr *NSReturnsAutoreleasedAttr::clone(ASTContext &C) const {
  auto *A = new (C) NSReturnsAutoreleasedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NSReturnsAutoreleasedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ns_returns_autoreleased))";
    break;
  }
  case 1 : {
    OS << " [[clang::ns_returns_autoreleased]]";
    break;
  }
  case 2 : {
    OS << " [[clang::ns_returns_autoreleased]]";
    break;
  }
}
}

const char *NSReturnsAutoreleasedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ns_returns_autoreleased";
  case 1:
    return "ns_returns_autoreleased";
  case 2:
    return "ns_returns_autoreleased";
  }
}

NSReturnsNotRetainedAttr *NSReturnsNotRetainedAttr::clone(ASTContext &C) const {
  auto *A = new (C) NSReturnsNotRetainedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NSReturnsNotRetainedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ns_returns_not_retained))";
    break;
  }
  case 1 : {
    OS << " [[clang::ns_returns_not_retained]]";
    break;
  }
  case 2 : {
    OS << " [[clang::ns_returns_not_retained]]";
    break;
  }
}
}

const char *NSReturnsNotRetainedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ns_returns_not_retained";
  case 1:
    return "ns_returns_not_retained";
  case 2:
    return "ns_returns_not_retained";
  }
}

NSReturnsRetainedAttr *NSReturnsRetainedAttr::clone(ASTContext &C) const {
  auto *A = new (C) NSReturnsRetainedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NSReturnsRetainedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ns_returns_retained))";
    break;
  }
  case 1 : {
    OS << " [[clang::ns_returns_retained]]";
    break;
  }
  case 2 : {
    OS << " [[clang::ns_returns_retained]]";
    break;
  }
}
}

const char *NSReturnsRetainedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ns_returns_retained";
  case 1:
    return "ns_returns_retained";
  case 2:
    return "ns_returns_retained";
  }
}

NakedAttr *NakedAttr::clone(ASTContext &C) const {
  auto *A = new (C) NakedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NakedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((naked))";
    break;
  }
  case 1 : {
    OS << " [[gnu::naked]]";
    break;
  }
  case 2 : {
    OS << " __declspec(naked)";
    break;
  }
}
}

const char *NakedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "naked";
  case 1:
    return "naked";
  case 2:
    return "naked";
  }
}

NoAliasAttr *NoAliasAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoAliasAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoAliasAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(noalias)";
    break;
  }
}
}

const char *NoAliasAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "noalias";
  }
}

NoBuiltinAttr *NoBuiltinAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoBuiltinAttr(C, *this, builtinNames_, builtinNames_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoBuiltinAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((no_builtin(";
  bool isFirst = true;
  for (const auto &Val : builtinNames()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << Val << "\"";
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::no_builtin(";
  bool isFirst = true;
  for (const auto &Val : builtinNames()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << Val << "\"";
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::no_builtin(";
  bool isFirst = true;
  for (const auto &Val : builtinNames()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << Val << "\"";
  }
  OS << ")]]";
    break;
  }
}
}

const char *NoBuiltinAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_builtin";
  case 1:
    return "no_builtin";
  case 2:
    return "no_builtin";
  }
}

NoCommonAttr *NoCommonAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoCommonAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoCommonAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((nocommon))";
    break;
  }
  case 1 : {
    OS << " [[gnu::nocommon]]";
    break;
  }
}
}

const char *NoCommonAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "nocommon";
  case 1:
    return "nocommon";
  }
}

NoDebugAttr *NoDebugAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoDebugAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoDebugAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((nodebug))";
    break;
  }
  case 1 : {
    OS << " [[gnu::nodebug]]";
    break;
  }
}
}

const char *NoDebugAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "nodebug";
  case 1:
    return "nodebug";
  }
}

NoDerefAttr *NoDerefAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoDerefAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoDerefAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((noderef))";
    break;
  }
  case 1 : {
    OS << " [[clang::noderef]]";
    break;
  }
  case 2 : {
    OS << " [[clang::noderef]]";
    break;
  }
}
}

const char *NoDerefAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "noderef";
  case 1:
    return "noderef";
  case 2:
    return "noderef";
  }
}

NoDestroyAttr *NoDestroyAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoDestroyAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoDestroyAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((no_destroy))";
    break;
  }
  case 1 : {
    OS << " [[clang::no_destroy]]";
    break;
  }
}
}

const char *NoDestroyAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_destroy";
  case 1:
    return "no_destroy";
  }
}

NoDuplicateAttr *NoDuplicateAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoDuplicateAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoDuplicateAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((noduplicate))";
    break;
  }
  case 1 : {
    OS << " [[clang::noduplicate]]";
    break;
  }
  case 2 : {
    OS << " [[clang::noduplicate]]";
    break;
  }
}
}

const char *NoDuplicateAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "noduplicate";
  case 1:
    return "noduplicate";
  case 2:
    return "noduplicate";
  }
}

NoEscapeAttr *NoEscapeAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoEscapeAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoEscapeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((noescape))";
    break;
  }
  case 1 : {
    OS << " [[clang::noescape]]";
    break;
  }
  case 2 : {
    OS << " [[clang::noescape]]";
    break;
  }
}
}

const char *NoEscapeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "noescape";
  case 1:
    return "noescape";
  case 2:
    return "noescape";
  }
}

NoInlineAttr *NoInlineAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoInlineAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoInlineAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((noinline))";
    break;
  }
  case 1 : {
    OS << " [[gnu::noinline]]";
    break;
  }
  case 2 : {
    OS << " __declspec(noinline)";
    break;
  }
}
}

const char *NoInlineAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "noinline";
  case 1:
    return "noinline";
  case 2:
    return "noinline";
  }
}

NoInstrumentFunctionAttr *NoInstrumentFunctionAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoInstrumentFunctionAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoInstrumentFunctionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((no_instrument_function))";
    break;
  }
  case 1 : {
    OS << " [[gnu::no_instrument_function]]";
    break;
  }
}
}

const char *NoInstrumentFunctionAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_instrument_function";
  case 1:
    return "no_instrument_function";
  }
}

NoMicroMipsAttr *NoMicroMipsAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoMicroMipsAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoMicroMipsAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((nomicromips))";
    break;
  }
  case 1 : {
    OS << " [[gnu::nomicromips]]";
    break;
  }
}
}

const char *NoMicroMipsAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "nomicromips";
  case 1:
    return "nomicromips";
  }
}

NoMips16Attr *NoMips16Attr::clone(ASTContext &C) const {
  auto *A = new (C) NoMips16Attr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoMips16Attr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((nomips16))";
    break;
  }
  case 1 : {
    OS << " [[gnu::nomips16]]";
    break;
  }
}
}

const char *NoMips16Attr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "nomips16";
  case 1:
    return "nomips16";
  }
}

NoReturnAttr *NoReturnAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoReturnAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoReturnAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((noreturn))";
    break;
  }
  case 1 : {
    OS << " [[gnu::noreturn]]";
    break;
  }
  case 2 : {
    OS << " __declspec(noreturn)";
    break;
  }
}
}

const char *NoReturnAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "noreturn";
  case 1:
    return "noreturn";
  case 2:
    return "noreturn";
  }
}

NoSanitizeAttr *NoSanitizeAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoSanitizeAttr(C, *this, sanitizers_, sanitizers_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoSanitizeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((no_sanitize(";
  bool isFirst = true;
  for (const auto &Val : sanitizers()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << Val << "\"";
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::no_sanitize(";
  bool isFirst = true;
  for (const auto &Val : sanitizers()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << Val << "\"";
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::no_sanitize(";
  bool isFirst = true;
  for (const auto &Val : sanitizers()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << Val << "\"";
  }
  OS << ")]]";
    break;
  }
}
}

const char *NoSanitizeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_sanitize";
  case 1:
    return "no_sanitize";
  case 2:
    return "no_sanitize";
  }
}

NoSpeculativeLoadHardeningAttr *NoSpeculativeLoadHardeningAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoSpeculativeLoadHardeningAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoSpeculativeLoadHardeningAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((no_speculative_load_hardening))";
    break;
  }
  case 1 : {
    OS << " [[clang::no_speculative_load_hardening]]";
    break;
  }
  case 2 : {
    OS << " [[clang::no_speculative_load_hardening]]";
    break;
  }
}
}

const char *NoSpeculativeLoadHardeningAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_speculative_load_hardening";
  case 1:
    return "no_speculative_load_hardening";
  case 2:
    return "no_speculative_load_hardening";
  }
}

NoSplitStackAttr *NoSplitStackAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoSplitStackAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoSplitStackAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((no_split_stack))";
    break;
  }
  case 1 : {
    OS << " [[gnu::no_split_stack]]";
    break;
  }
}
}

const char *NoSplitStackAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_split_stack";
  case 1:
    return "no_split_stack";
  }
}

NoStackProtectorAttr *NoStackProtectorAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoStackProtectorAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoStackProtectorAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((no_stack_protector))";
    break;
  }
  case 1 : {
    OS << " [[clang::no_stack_protector]]";
    break;
  }
  case 2 : {
    OS << " [[clang::no_stack_protector]]";
    break;
  }
}
}

const char *NoStackProtectorAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_stack_protector";
  case 1:
    return "no_stack_protector";
  case 2:
    return "no_stack_protector";
  }
}

NoThreadSafetyAnalysisAttr *NoThreadSafetyAnalysisAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoThreadSafetyAnalysisAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoThreadSafetyAnalysisAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((no_thread_safety_analysis))";
    break;
  }
  case 1 : {
    OS << " [[clang::no_thread_safety_analysis]]";
    break;
  }
  case 2 : {
    OS << " [[clang::no_thread_safety_analysis]]";
    break;
  }
}
}

const char *NoThreadSafetyAnalysisAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_thread_safety_analysis";
  case 1:
    return "no_thread_safety_analysis";
  case 2:
    return "no_thread_safety_analysis";
  }
}

NoThrowAttr *NoThrowAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoThrowAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoThrowAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((nothrow))";
    break;
  }
  case 1 : {
    OS << " [[gnu::nothrow]]";
    break;
  }
  case 2 : {
    OS << " __declspec(nothrow)";
    break;
  }
}
}

const char *NoThrowAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "nothrow";
  case 1:
    return "nothrow";
  case 2:
    return "nothrow";
  }
}

NoUniqueAddressAttr *NoUniqueAddressAttr::clone(ASTContext &C) const {
  auto *A = new (C) NoUniqueAddressAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NoUniqueAddressAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " [[no_unique_address]]";
    break;
  }
}
}

const char *NoUniqueAddressAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "no_unique_address";
  }
}

NonNullAttr *NonNullAttr::clone(ASTContext &C) const {
  auto *A = new (C) NonNullAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NonNullAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((nonnull(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::nonnull(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")]]";
    break;
  }
}
}

const char *NonNullAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "nonnull";
  case 1:
    return "nonnull";
  }
}

NotTailCalledAttr *NotTailCalledAttr::clone(ASTContext &C) const {
  auto *A = new (C) NotTailCalledAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void NotTailCalledAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((not_tail_called))";
    break;
  }
  case 1 : {
    OS << " [[clang::not_tail_called]]";
    break;
  }
  case 2 : {
    OS << " [[clang::not_tail_called]]";
    break;
  }
}
}

const char *NotTailCalledAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "not_tail_called";
  case 1:
    return "not_tail_called";
  case 2:
    return "not_tail_called";
  }
}

OMPAllocateDeclAttr *OMPAllocateDeclAttr::clone(ASTContext &C) const {
  auto *A = new (C) OMPAllocateDeclAttr(C, *this, allocatorType, allocator);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OMPAllocateDeclAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *OMPAllocateDeclAttr::getSpelling() const {
  return "(No spelling)";
}

OMPCaptureKindAttr *OMPCaptureKindAttr::clone(ASTContext &C) const {
  auto *A = new (C) OMPCaptureKindAttr(C, *this, captureKind);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OMPCaptureKindAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *OMPCaptureKindAttr::getSpelling() const {
  return "(No spelling)";
}

OMPCaptureNoInitAttr *OMPCaptureNoInitAttr::clone(ASTContext &C) const {
  auto *A = new (C) OMPCaptureNoInitAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OMPCaptureNoInitAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *OMPCaptureNoInitAttr::getSpelling() const {
  return "(No spelling)";
}

OMPDeclareSimdDeclAttr *OMPDeclareSimdDeclAttr::clone(ASTContext &C) const {
  auto *A = new (C) OMPDeclareSimdDeclAttr(C, *this, branchState, simdlen, uniforms_, uniforms_Size, aligneds_, aligneds_Size, alignments_, alignments_Size, linears_, linears_Size, modifiers_, modifiers_Size, steps_, steps_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OMPDeclareSimdDeclAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << "#pragma omp declare simd";
    printPrettyPragma(OS, Policy);
    OS << "\n";    break;
  }
}
}

const char *OMPDeclareSimdDeclAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "declare simd";
  }
}

OMPDeclareTargetDeclAttr *OMPDeclareTargetDeclAttr::clone(ASTContext &C) const {
  auto *A = new (C) OMPDeclareTargetDeclAttr(C, *this, mapType, devType);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OMPDeclareTargetDeclAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << "#pragma omp declare target";
    printPrettyPragma(OS, Policy);
    OS << "\n";    break;
  }
}
}

const char *OMPDeclareTargetDeclAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "declare target";
  }
}

OMPDeclareVariantAttr *OMPDeclareVariantAttr::clone(ASTContext &C) const {
  auto *A = new (C) OMPDeclareVariantAttr(C, *this, variantFuncRef, scores_, scores_Size, ctxSelectorSets_, ctxSelectorSets_Size, ctxSelectors_, ctxSelectors_Size, implVendors_, implVendors_Size, deviceKinds_, deviceKinds_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OMPDeclareVariantAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << "#pragma omp declare variant";
    printPrettyPragma(OS, Policy);
    OS << "\n";    break;
  }
}
}

const char *OMPDeclareVariantAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "declare variant";
  }
}

OMPReferencedVarAttr *OMPReferencedVarAttr::clone(ASTContext &C) const {
  auto *A = new (C) OMPReferencedVarAttr(C, *this, ref);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OMPReferencedVarAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *OMPReferencedVarAttr::getSpelling() const {
  return "(No spelling)";
}

OMPThreadPrivateDeclAttr *OMPThreadPrivateDeclAttr::clone(ASTContext &C) const {
  auto *A = new (C) OMPThreadPrivateDeclAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OMPThreadPrivateDeclAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *OMPThreadPrivateDeclAttr::getSpelling() const {
  return "(No spelling)";
}

OSConsumedAttr *OSConsumedAttr::clone(ASTContext &C) const {
  auto *A = new (C) OSConsumedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OSConsumedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((os_consumed))";
    break;
  }
  case 1 : {
    OS << " [[clang::os_consumed]]";
    break;
  }
  case 2 : {
    OS << " [[clang::os_consumed]]";
    break;
  }
}
}

const char *OSConsumedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "os_consumed";
  case 1:
    return "os_consumed";
  case 2:
    return "os_consumed";
  }
}

OSConsumesThisAttr *OSConsumesThisAttr::clone(ASTContext &C) const {
  auto *A = new (C) OSConsumesThisAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OSConsumesThisAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((os_consumes_this))";
    break;
  }
  case 1 : {
    OS << " [[clang::os_consumes_this]]";
    break;
  }
  case 2 : {
    OS << " [[clang::os_consumes_this]]";
    break;
  }
}
}

const char *OSConsumesThisAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "os_consumes_this";
  case 1:
    return "os_consumes_this";
  case 2:
    return "os_consumes_this";
  }
}

OSReturnsNotRetainedAttr *OSReturnsNotRetainedAttr::clone(ASTContext &C) const {
  auto *A = new (C) OSReturnsNotRetainedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OSReturnsNotRetainedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((os_returns_not_retained))";
    break;
  }
  case 1 : {
    OS << " [[clang::os_returns_not_retained]]";
    break;
  }
  case 2 : {
    OS << " [[clang::os_returns_not_retained]]";
    break;
  }
}
}

const char *OSReturnsNotRetainedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "os_returns_not_retained";
  case 1:
    return "os_returns_not_retained";
  case 2:
    return "os_returns_not_retained";
  }
}

OSReturnsRetainedAttr *OSReturnsRetainedAttr::clone(ASTContext &C) const {
  auto *A = new (C) OSReturnsRetainedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OSReturnsRetainedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((os_returns_retained))";
    break;
  }
  case 1 : {
    OS << " [[clang::os_returns_retained]]";
    break;
  }
  case 2 : {
    OS << " [[clang::os_returns_retained]]";
    break;
  }
}
}

const char *OSReturnsRetainedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "os_returns_retained";
  case 1:
    return "os_returns_retained";
  case 2:
    return "os_returns_retained";
  }
}

OSReturnsRetainedOnNonZeroAttr *OSReturnsRetainedOnNonZeroAttr::clone(ASTContext &C) const {
  auto *A = new (C) OSReturnsRetainedOnNonZeroAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OSReturnsRetainedOnNonZeroAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((os_returns_retained_on_non_zero))";
    break;
  }
  case 1 : {
    OS << " [[clang::os_returns_retained_on_non_zero]]";
    break;
  }
  case 2 : {
    OS << " [[clang::os_returns_retained_on_non_zero]]";
    break;
  }
}
}

const char *OSReturnsRetainedOnNonZeroAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "os_returns_retained_on_non_zero";
  case 1:
    return "os_returns_retained_on_non_zero";
  case 2:
    return "os_returns_retained_on_non_zero";
  }
}

OSReturnsRetainedOnZeroAttr *OSReturnsRetainedOnZeroAttr::clone(ASTContext &C) const {
  auto *A = new (C) OSReturnsRetainedOnZeroAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OSReturnsRetainedOnZeroAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((os_returns_retained_on_zero))";
    break;
  }
  case 1 : {
    OS << " [[clang::os_returns_retained_on_zero]]";
    break;
  }
  case 2 : {
    OS << " [[clang::os_returns_retained_on_zero]]";
    break;
  }
}
}

const char *OSReturnsRetainedOnZeroAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "os_returns_retained_on_zero";
  case 1:
    return "os_returns_retained_on_zero";
  case 2:
    return "os_returns_retained_on_zero";
  }
}

ObjCBoxableAttr *ObjCBoxableAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCBoxableAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCBoxableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_boxable))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_boxable]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_boxable]]";
    break;
  }
}
}

const char *ObjCBoxableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_boxable";
  case 1:
    return "objc_boxable";
  case 2:
    return "objc_boxable";
  }
}

ObjCBridgeAttr *ObjCBridgeAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCBridgeAttr(C, *this, bridgedType);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCBridgeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_bridge(" << (getBridgedType() ? getBridgedType()->getName() : "") << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_bridge(" << (getBridgedType() ? getBridgedType()->getName() : "") << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_bridge(" << (getBridgedType() ? getBridgedType()->getName() : "") << ")]]";
    break;
  }
}
}

const char *ObjCBridgeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_bridge";
  case 1:
    return "objc_bridge";
  case 2:
    return "objc_bridge";
  }
}

ObjCBridgeMutableAttr *ObjCBridgeMutableAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCBridgeMutableAttr(C, *this, bridgedType);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCBridgeMutableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_bridge_mutable(" << (getBridgedType() ? getBridgedType()->getName() : "") << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_bridge_mutable(" << (getBridgedType() ? getBridgedType()->getName() : "") << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_bridge_mutable(" << (getBridgedType() ? getBridgedType()->getName() : "") << ")]]";
    break;
  }
}
}

const char *ObjCBridgeMutableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_bridge_mutable";
  case 1:
    return "objc_bridge_mutable";
  case 2:
    return "objc_bridge_mutable";
  }
}

ObjCBridgeRelatedAttr *ObjCBridgeRelatedAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCBridgeRelatedAttr(C, *this, relatedClass, classMethod, instanceMethod);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCBridgeRelatedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_bridge_related(" << (getRelatedClass() ? getRelatedClass()->getName() : "") << ", " << (getClassMethod() ? getClassMethod()->getName() : "") << ", " << (getInstanceMethod() ? getInstanceMethod()->getName() : "") << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_bridge_related(" << (getRelatedClass() ? getRelatedClass()->getName() : "") << ", " << (getClassMethod() ? getClassMethod()->getName() : "") << ", " << (getInstanceMethod() ? getInstanceMethod()->getName() : "") << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_bridge_related(" << (getRelatedClass() ? getRelatedClass()->getName() : "") << ", " << (getClassMethod() ? getClassMethod()->getName() : "") << ", " << (getInstanceMethod() ? getInstanceMethod()->getName() : "") << ")]]";
    break;
  }
}
}

const char *ObjCBridgeRelatedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_bridge_related";
  case 1:
    return "objc_bridge_related";
  case 2:
    return "objc_bridge_related";
  }
}

ObjCClassStubAttr *ObjCClassStubAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCClassStubAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCClassStubAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_class_stub))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_class_stub]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_class_stub]]";
    break;
  }
}
}

const char *ObjCClassStubAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_class_stub";
  case 1:
    return "objc_class_stub";
  case 2:
    return "objc_class_stub";
  }
}

ObjCDesignatedInitializerAttr *ObjCDesignatedInitializerAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCDesignatedInitializerAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCDesignatedInitializerAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_designated_initializer))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_designated_initializer]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_designated_initializer]]";
    break;
  }
}
}

const char *ObjCDesignatedInitializerAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_designated_initializer";
  case 1:
    return "objc_designated_initializer";
  case 2:
    return "objc_designated_initializer";
  }
}

ObjCDirectAttr *ObjCDirectAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCDirectAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCDirectAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_direct))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_direct]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_direct]]";
    break;
  }
}
}

const char *ObjCDirectAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_direct";
  case 1:
    return "objc_direct";
  case 2:
    return "objc_direct";
  }
}

ObjCDirectMembersAttr *ObjCDirectMembersAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCDirectMembersAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCDirectMembersAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_direct_members))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_direct_members]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_direct_members]]";
    break;
  }
}
}

const char *ObjCDirectMembersAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_direct_members";
  case 1:
    return "objc_direct_members";
  case 2:
    return "objc_direct_members";
  }
}

ObjCExceptionAttr *ObjCExceptionAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCExceptionAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCExceptionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_exception))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_exception]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_exception]]";
    break;
  }
}
}

const char *ObjCExceptionAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_exception";
  case 1:
    return "objc_exception";
  case 2:
    return "objc_exception";
  }
}

ObjCExplicitProtocolImplAttr *ObjCExplicitProtocolImplAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCExplicitProtocolImplAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCExplicitProtocolImplAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_protocol_requires_explicit_implementation))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_protocol_requires_explicit_implementation]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_protocol_requires_explicit_implementation]]";
    break;
  }
}
}

const char *ObjCExplicitProtocolImplAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_protocol_requires_explicit_implementation";
  case 1:
    return "objc_protocol_requires_explicit_implementation";
  case 2:
    return "objc_protocol_requires_explicit_implementation";
  }
}

ObjCExternallyRetainedAttr *ObjCExternallyRetainedAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCExternallyRetainedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCExternallyRetainedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_externally_retained))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_externally_retained]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_externally_retained]]";
    break;
  }
}
}

const char *ObjCExternallyRetainedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_externally_retained";
  case 1:
    return "objc_externally_retained";
  case 2:
    return "objc_externally_retained";
  }
}

ObjCGCAttr *ObjCGCAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCGCAttr(C, *this, kind);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCGCAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_gc(" << (getKind() ? getKind()->getName() : "") << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_gc(" << (getKind() ? getKind()->getName() : "") << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_gc(" << (getKind() ? getKind()->getName() : "") << ")]]";
    break;
  }
}
}

const char *ObjCGCAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_gc";
  case 1:
    return "objc_gc";
  case 2:
    return "objc_gc";
  }
}

ObjCIndependentClassAttr *ObjCIndependentClassAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCIndependentClassAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCIndependentClassAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_independent_class))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_independent_class]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_independent_class]]";
    break;
  }
}
}

const char *ObjCIndependentClassAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_independent_class";
  case 1:
    return "objc_independent_class";
  case 2:
    return "objc_independent_class";
  }
}

ObjCInertUnsafeUnretainedAttr *ObjCInertUnsafeUnretainedAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCInertUnsafeUnretainedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCInertUnsafeUnretainedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __unsafe_unretained";
    break;
  }
}
}

const char *ObjCInertUnsafeUnretainedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__unsafe_unretained";
  }
}

ObjCKindOfAttr *ObjCKindOfAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCKindOfAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCKindOfAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __kindof";
    break;
  }
}
}

const char *ObjCKindOfAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__kindof";
  }
}

ObjCMethodFamilyAttr *ObjCMethodFamilyAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCMethodFamilyAttr(C, *this, family);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCMethodFamilyAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_method_family(\"" << ObjCMethodFamilyAttr::ConvertFamilyKindToStr(getFamily()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_method_family(\"" << ObjCMethodFamilyAttr::ConvertFamilyKindToStr(getFamily()) << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_method_family(\"" << ObjCMethodFamilyAttr::ConvertFamilyKindToStr(getFamily()) << "\")]]";
    break;
  }
}
}

const char *ObjCMethodFamilyAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_method_family";
  case 1:
    return "objc_method_family";
  case 2:
    return "objc_method_family";
  }
}

ObjCNSObjectAttr *ObjCNSObjectAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCNSObjectAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCNSObjectAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((NSObject))";
    break;
  }
  case 1 : {
    OS << " [[clang::NSObject]]";
    break;
  }
  case 2 : {
    OS << " [[clang::NSObject]]";
    break;
  }
}
}

const char *ObjCNSObjectAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "NSObject";
  case 1:
    return "NSObject";
  case 2:
    return "NSObject";
  }
}

ObjCNonLazyClassAttr *ObjCNonLazyClassAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCNonLazyClassAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCNonLazyClassAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_nonlazy_class))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_nonlazy_class]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_nonlazy_class]]";
    break;
  }
}
}

const char *ObjCNonLazyClassAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_nonlazy_class";
  case 1:
    return "objc_nonlazy_class";
  case 2:
    return "objc_nonlazy_class";
  }
}

ObjCOwnershipAttr *ObjCOwnershipAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCOwnershipAttr(C, *this, kind);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCOwnershipAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_ownership(" << (getKind() ? getKind()->getName() : "") << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_ownership(" << (getKind() ? getKind()->getName() : "") << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_ownership(" << (getKind() ? getKind()->getName() : "") << ")]]";
    break;
  }
}
}

const char *ObjCOwnershipAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_ownership";
  case 1:
    return "objc_ownership";
  case 2:
    return "objc_ownership";
  }
}

ObjCPreciseLifetimeAttr *ObjCPreciseLifetimeAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCPreciseLifetimeAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCPreciseLifetimeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_precise_lifetime))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_precise_lifetime]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_precise_lifetime]]";
    break;
  }
}
}

const char *ObjCPreciseLifetimeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_precise_lifetime";
  case 1:
    return "objc_precise_lifetime";
  case 2:
    return "objc_precise_lifetime";
  }
}

ObjCRequiresPropertyDefsAttr *ObjCRequiresPropertyDefsAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCRequiresPropertyDefsAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCRequiresPropertyDefsAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_requires_property_definitions))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_requires_property_definitions]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_requires_property_definitions]]";
    break;
  }
}
}

const char *ObjCRequiresPropertyDefsAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_requires_property_definitions";
  case 1:
    return "objc_requires_property_definitions";
  case 2:
    return "objc_requires_property_definitions";
  }
}

ObjCRequiresSuperAttr *ObjCRequiresSuperAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCRequiresSuperAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCRequiresSuperAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_requires_super))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_requires_super]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_requires_super]]";
    break;
  }
}
}

const char *ObjCRequiresSuperAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_requires_super";
  case 1:
    return "objc_requires_super";
  case 2:
    return "objc_requires_super";
  }
}

ObjCReturnsInnerPointerAttr *ObjCReturnsInnerPointerAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCReturnsInnerPointerAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCReturnsInnerPointerAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_returns_inner_pointer))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_returns_inner_pointer]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_returns_inner_pointer]]";
    break;
  }
}
}

const char *ObjCReturnsInnerPointerAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_returns_inner_pointer";
  case 1:
    return "objc_returns_inner_pointer";
  case 2:
    return "objc_returns_inner_pointer";
  }
}

ObjCRootClassAttr *ObjCRootClassAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCRootClassAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCRootClassAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_root_class))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_root_class]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_root_class]]";
    break;
  }
}
}

const char *ObjCRootClassAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_root_class";
  case 1:
    return "objc_root_class";
  case 2:
    return "objc_root_class";
  }
}

ObjCRuntimeNameAttr *ObjCRuntimeNameAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCRuntimeNameAttr(C, *this, getMetadataName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCRuntimeNameAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_runtime_name(\"" << getMetadataName() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_runtime_name(\"" << getMetadataName() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_runtime_name(\"" << getMetadataName() << "\")]]";
    break;
  }
}
}

const char *ObjCRuntimeNameAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_runtime_name";
  case 1:
    return "objc_runtime_name";
  case 2:
    return "objc_runtime_name";
  }
}

ObjCRuntimeVisibleAttr *ObjCRuntimeVisibleAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCRuntimeVisibleAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCRuntimeVisibleAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_runtime_visible))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_runtime_visible]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_runtime_visible]]";
    break;
  }
}
}

const char *ObjCRuntimeVisibleAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_runtime_visible";
  case 1:
    return "objc_runtime_visible";
  case 2:
    return "objc_runtime_visible";
  }
}

ObjCSubclassingRestrictedAttr *ObjCSubclassingRestrictedAttr::clone(ASTContext &C) const {
  auto *A = new (C) ObjCSubclassingRestrictedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ObjCSubclassingRestrictedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((objc_subclassing_restricted))";
    break;
  }
  case 1 : {
    OS << " [[clang::objc_subclassing_restricted]]";
    break;
  }
  case 2 : {
    OS << " [[clang::objc_subclassing_restricted]]";
    break;
  }
}
}

const char *ObjCSubclassingRestrictedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "objc_subclassing_restricted";
  case 1:
    return "objc_subclassing_restricted";
  case 2:
    return "objc_subclassing_restricted";
  }
}

OpenCLAccessAttr *OpenCLAccessAttr::clone(ASTContext &C) const {
  auto *A = new (C) OpenCLAccessAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OpenCLAccessAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __read_only";
    break;
  }
  case 1 : {
    OS << " read_only";
    break;
  }
  case 2 : {
    OS << " __write_only";
    break;
  }
  case 3 : {
    OS << " write_only";
    break;
  }
  case 4 : {
    OS << " __read_write";
    break;
  }
  case 5 : {
    OS << " read_write";
    break;
  }
}
}

const char *OpenCLAccessAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__read_only";
  case 1:
    return "read_only";
  case 2:
    return "__write_only";
  case 3:
    return "write_only";
  case 4:
    return "__read_write";
  case 5:
    return "read_write";
  }
}

OpenCLConstantAddressSpaceAttr *OpenCLConstantAddressSpaceAttr::clone(ASTContext &C) const {
  auto *A = new (C) OpenCLConstantAddressSpaceAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OpenCLConstantAddressSpaceAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __constant";
    break;
  }
  case 1 : {
    OS << " constant";
    break;
  }
  case 2 : {
    OS << " __attribute__((opencl_constant))";
    break;
  }
  case 3 : {
    OS << " [[clang::opencl_constant]]";
    break;
  }
  case 4 : {
    OS << " [[clang::opencl_constant]]";
    break;
  }
}
}

const char *OpenCLConstantAddressSpaceAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__constant";
  case 1:
    return "constant";
  case 2:
    return "opencl_constant";
  case 3:
    return "opencl_constant";
  case 4:
    return "opencl_constant";
  }
}

OpenCLGenericAddressSpaceAttr *OpenCLGenericAddressSpaceAttr::clone(ASTContext &C) const {
  auto *A = new (C) OpenCLGenericAddressSpaceAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OpenCLGenericAddressSpaceAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __generic";
    break;
  }
  case 1 : {
    OS << " generic";
    break;
  }
  case 2 : {
    OS << " __attribute__((opencl_generic))";
    break;
  }
  case 3 : {
    OS << " [[clang::opencl_generic]]";
    break;
  }
  case 4 : {
    OS << " [[clang::opencl_generic]]";
    break;
  }
}
}

const char *OpenCLGenericAddressSpaceAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__generic";
  case 1:
    return "generic";
  case 2:
    return "opencl_generic";
  case 3:
    return "opencl_generic";
  case 4:
    return "opencl_generic";
  }
}

OpenCLGlobalAddressSpaceAttr *OpenCLGlobalAddressSpaceAttr::clone(ASTContext &C) const {
  auto *A = new (C) OpenCLGlobalAddressSpaceAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OpenCLGlobalAddressSpaceAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __global";
    break;
  }
  case 1 : {
    OS << " global";
    break;
  }
  case 2 : {
    OS << " __attribute__((opencl_global))";
    break;
  }
  case 3 : {
    OS << " [[clang::opencl_global]]";
    break;
  }
  case 4 : {
    OS << " [[clang::opencl_global]]";
    break;
  }
}
}

const char *OpenCLGlobalAddressSpaceAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__global";
  case 1:
    return "global";
  case 2:
    return "opencl_global";
  case 3:
    return "opencl_global";
  case 4:
    return "opencl_global";
  }
}

OpenCLIntelReqdSubGroupSizeAttr *OpenCLIntelReqdSubGroupSizeAttr::clone(ASTContext &C) const {
  auto *A = new (C) OpenCLIntelReqdSubGroupSizeAttr(C, *this, subGroupSize);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OpenCLIntelReqdSubGroupSizeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((intel_reqd_sub_group_size(" << getSubGroupSize() << ")))";
    break;
  }
}
}

const char *OpenCLIntelReqdSubGroupSizeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "intel_reqd_sub_group_size";
  }
}

OpenCLKernelAttr *OpenCLKernelAttr::clone(ASTContext &C) const {
  auto *A = new (C) OpenCLKernelAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OpenCLKernelAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __kernel";
    break;
  }
  case 1 : {
    OS << " kernel";
    break;
  }
}
}

const char *OpenCLKernelAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__kernel";
  case 1:
    return "kernel";
  }
}

OpenCLLocalAddressSpaceAttr *OpenCLLocalAddressSpaceAttr::clone(ASTContext &C) const {
  auto *A = new (C) OpenCLLocalAddressSpaceAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OpenCLLocalAddressSpaceAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __local";
    break;
  }
  case 1 : {
    OS << " local";
    break;
  }
  case 2 : {
    OS << " __attribute__((opencl_local))";
    break;
  }
  case 3 : {
    OS << " [[clang::opencl_local]]";
    break;
  }
  case 4 : {
    OS << " [[clang::opencl_local]]";
    break;
  }
}
}

const char *OpenCLLocalAddressSpaceAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__local";
  case 1:
    return "local";
  case 2:
    return "opencl_local";
  case 3:
    return "opencl_local";
  case 4:
    return "opencl_local";
  }
}

OpenCLPrivateAddressSpaceAttr *OpenCLPrivateAddressSpaceAttr::clone(ASTContext &C) const {
  auto *A = new (C) OpenCLPrivateAddressSpaceAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OpenCLPrivateAddressSpaceAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __private";
    break;
  }
  case 1 : {
    OS << " private";
    break;
  }
  case 2 : {
    OS << " __attribute__((opencl_private))";
    break;
  }
  case 3 : {
    OS << " [[clang::opencl_private]]";
    break;
  }
  case 4 : {
    OS << " [[clang::opencl_private]]";
    break;
  }
}
}

const char *OpenCLPrivateAddressSpaceAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__private";
  case 1:
    return "private";
  case 2:
    return "opencl_private";
  case 3:
    return "opencl_private";
  case 4:
    return "opencl_private";
  }
}

OpenCLUnrollHintAttr *OpenCLUnrollHintAttr::clone(ASTContext &C) const {
  auto *A = new (C) OpenCLUnrollHintAttr(C, *this, unrollHint);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OpenCLUnrollHintAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((opencl_unroll_hint(" << getUnrollHint() << ")))";
    break;
  }
}
}

const char *OpenCLUnrollHintAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "opencl_unroll_hint";
  }
}

OptimizeNoneAttr *OptimizeNoneAttr::clone(ASTContext &C) const {
  auto *A = new (C) OptimizeNoneAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OptimizeNoneAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((optnone))";
    break;
  }
  case 1 : {
    OS << " [[clang::optnone]]";
    break;
  }
  case 2 : {
    OS << " [[clang::optnone]]";
    break;
  }
}
}

const char *OptimizeNoneAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "optnone";
  case 1:
    return "optnone";
  case 2:
    return "optnone";
  }
}

OverloadableAttr *OverloadableAttr::clone(ASTContext &C) const {
  auto *A = new (C) OverloadableAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OverloadableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((overloadable))";
    break;
  }
  case 1 : {
    OS << " [[clang::overloadable]]";
    break;
  }
  case 2 : {
    OS << " [[clang::overloadable]]";
    break;
  }
}
}

const char *OverloadableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "overloadable";
  case 1:
    return "overloadable";
  case 2:
    return "overloadable";
  }
}

OverrideAttr *OverrideAttr::clone(ASTContext &C) const {
  auto *A = new (C) OverrideAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OverrideAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " override";
    break;
  }
}
}

const char *OverrideAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "override";
  }
}

OwnerAttr *OwnerAttr::clone(ASTContext &C) const {
  auto *A = new (C) OwnerAttr(C, *this, derefType);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OwnerAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " [[gsl::Owner";
    unsigned TrailingOmittedArgs = 0;
    if (!getDerefTypeLoc())
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!getDerefTypeLoc())) {
      OS << "" << getDerefType().getAsString() << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << "]]";
    break;
  }
}
}

const char *OwnerAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "Owner";
  }
}

OwnershipAttr *OwnershipAttr::clone(ASTContext &C) const {
  auto *A = new (C) OwnershipAttr(C, *this, module, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void OwnershipAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((ownership_holds(" << (getModule() ? getModule()->getName() : "") << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::ownership_holds(" << (getModule() ? getModule()->getName() : "") << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::ownership_holds(" << (getModule() ? getModule()->getName() : "") << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")]]";
    break;
  }
  case 3 : {
    OS << " __attribute__((ownership_returns(" << (getModule() ? getModule()->getName() : "") << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")))";
    break;
  }
  case 4 : {
    OS << " [[clang::ownership_returns(" << (getModule() ? getModule()->getName() : "") << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")]]";
    break;
  }
  case 5 : {
    OS << " [[clang::ownership_returns(" << (getModule() ? getModule()->getName() : "") << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")]]";
    break;
  }
  case 6 : {
    OS << " __attribute__((ownership_takes(" << (getModule() ? getModule()->getName() : "") << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")))";
    break;
  }
  case 7 : {
    OS << " [[clang::ownership_takes(" << (getModule() ? getModule()->getName() : "") << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")]]";
    break;
  }
  case 8 : {
    OS << " [[clang::ownership_takes(" << (getModule() ? getModule()->getName() : "") << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val.getSourceIndex();
  }
  OS << ")]]";
    break;
  }
}
}

const char *OwnershipAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "ownership_holds";
  case 1:
    return "ownership_holds";
  case 2:
    return "ownership_holds";
  case 3:
    return "ownership_returns";
  case 4:
    return "ownership_returns";
  case 5:
    return "ownership_returns";
  case 6:
    return "ownership_takes";
  case 7:
    return "ownership_takes";
  case 8:
    return "ownership_takes";
  }
}

PackedAttr *PackedAttr::clone(ASTContext &C) const {
  auto *A = new (C) PackedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PackedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((packed))";
    break;
  }
  case 1 : {
    OS << " [[gnu::packed]]";
    break;
  }
}
}

const char *PackedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "packed";
  case 1:
    return "packed";
  }
}

ParamTypestateAttr *ParamTypestateAttr::clone(ASTContext &C) const {
  auto *A = new (C) ParamTypestateAttr(C, *this, paramState);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ParamTypestateAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((param_typestate(\"" << ParamTypestateAttr::ConvertConsumedStateToStr(getParamState()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::param_typestate(\"" << ParamTypestateAttr::ConvertConsumedStateToStr(getParamState()) << "\")]]";
    break;
  }
}
}

const char *ParamTypestateAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "param_typestate";
  case 1:
    return "param_typestate";
  }
}

PascalAttr *PascalAttr::clone(ASTContext &C) const {
  auto *A = new (C) PascalAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PascalAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((pascal))";
    break;
  }
  case 1 : {
    OS << " [[clang::pascal]]";
    break;
  }
  case 2 : {
    OS << " [[clang::pascal]]";
    break;
  }
  case 3 : {
    OS << " __pascal";
    break;
  }
  case 4 : {
    OS << " _pascal";
    break;
  }
}
}

const char *PascalAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "pascal";
  case 1:
    return "pascal";
  case 2:
    return "pascal";
  case 3:
    return "__pascal";
  case 4:
    return "_pascal";
  }
}

PassObjectSizeAttr *PassObjectSizeAttr::clone(ASTContext &C) const {
  auto *A = new (C) PassObjectSizeAttr(C, *this, type);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PassObjectSizeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((pass_object_size(" << getType() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::pass_object_size(" << getType() << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::pass_object_size(" << getType() << ")]]";
    break;
  }
  case 3 : {
    OS << " __attribute__((pass_dynamic_object_size(" << getType() << ")))";
    break;
  }
  case 4 : {
    OS << " [[clang::pass_dynamic_object_size(" << getType() << ")]]";
    break;
  }
  case 5 : {
    OS << " [[clang::pass_dynamic_object_size(" << getType() << ")]]";
    break;
  }
}
}

const char *PassObjectSizeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "pass_object_size";
  case 1:
    return "pass_object_size";
  case 2:
    return "pass_object_size";
  case 3:
    return "pass_dynamic_object_size";
  case 4:
    return "pass_dynamic_object_size";
  case 5:
    return "pass_dynamic_object_size";
  }
}

PatchableFunctionEntryAttr *PatchableFunctionEntryAttr::clone(ASTContext &C) const {
  auto *A = new (C) PatchableFunctionEntryAttr(C, *this, count, offset);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PatchableFunctionEntryAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((patchable_function_entry(" << getCount() << ", " << getOffset() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::patchable_function_entry(" << getCount() << ", " << getOffset() << ")]]";
    break;
  }
}
}

const char *PatchableFunctionEntryAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "patchable_function_entry";
  case 1:
    return "patchable_function_entry";
  }
}

PcsAttr *PcsAttr::clone(ASTContext &C) const {
  auto *A = new (C) PcsAttr(C, *this, pCS);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PcsAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((pcs(\"" << PcsAttr::ConvertPCSTypeToStr(getPCS()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::pcs(\"" << PcsAttr::ConvertPCSTypeToStr(getPCS()) << "\")]]";
    break;
  }
}
}

const char *PcsAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "pcs";
  case 1:
    return "pcs";
  }
}

PointerAttr *PointerAttr::clone(ASTContext &C) const {
  auto *A = new (C) PointerAttr(C, *this, derefType);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PointerAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " [[gsl::Pointer";
    unsigned TrailingOmittedArgs = 0;
    if (!getDerefTypeLoc())
      ++TrailingOmittedArgs;
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << "(";
    OS << "";
    if (!(!getDerefTypeLoc())) {
      OS << "" << getDerefType().getAsString() << "";
    }
    OS << "";
    if (TrailingOmittedArgs < 1)
       OS << ")";
    OS << "]]";
    break;
  }
}
}

const char *PointerAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "Pointer";
  }
}

PragmaClangBSSSectionAttr *PragmaClangBSSSectionAttr::clone(ASTContext &C) const {
  auto *A = new (C) PragmaClangBSSSectionAttr(C, *this, getName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PragmaClangBSSSectionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *PragmaClangBSSSectionAttr::getSpelling() const {
  return "(No spelling)";
}

PragmaClangDataSectionAttr *PragmaClangDataSectionAttr::clone(ASTContext &C) const {
  auto *A = new (C) PragmaClangDataSectionAttr(C, *this, getName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PragmaClangDataSectionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *PragmaClangDataSectionAttr::getSpelling() const {
  return "(No spelling)";
}

PragmaClangRelroSectionAttr *PragmaClangRelroSectionAttr::clone(ASTContext &C) const {
  auto *A = new (C) PragmaClangRelroSectionAttr(C, *this, getName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PragmaClangRelroSectionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *PragmaClangRelroSectionAttr::getSpelling() const {
  return "(No spelling)";
}

PragmaClangRodataSectionAttr *PragmaClangRodataSectionAttr::clone(ASTContext &C) const {
  auto *A = new (C) PragmaClangRodataSectionAttr(C, *this, getName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PragmaClangRodataSectionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *PragmaClangRodataSectionAttr::getSpelling() const {
  return "(No spelling)";
}

PragmaClangTextSectionAttr *PragmaClangTextSectionAttr::clone(ASTContext &C) const {
  auto *A = new (C) PragmaClangTextSectionAttr(C, *this, getName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PragmaClangTextSectionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
}

const char *PragmaClangTextSectionAttr::getSpelling() const {
  return "(No spelling)";
}

PreserveAllAttr *PreserveAllAttr::clone(ASTContext &C) const {
  auto *A = new (C) PreserveAllAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PreserveAllAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((preserve_all))";
    break;
  }
  case 1 : {
    OS << " [[clang::preserve_all]]";
    break;
  }
  case 2 : {
    OS << " [[clang::preserve_all]]";
    break;
  }
}
}

const char *PreserveAllAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "preserve_all";
  case 1:
    return "preserve_all";
  case 2:
    return "preserve_all";
  }
}

PreserveMostAttr *PreserveMostAttr::clone(ASTContext &C) const {
  auto *A = new (C) PreserveMostAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PreserveMostAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((preserve_most))";
    break;
  }
  case 1 : {
    OS << " [[clang::preserve_most]]";
    break;
  }
  case 2 : {
    OS << " [[clang::preserve_most]]";
    break;
  }
}
}

const char *PreserveMostAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "preserve_most";
  case 1:
    return "preserve_most";
  case 2:
    return "preserve_most";
  }
}

PtGuardedByAttr *PtGuardedByAttr::clone(ASTContext &C) const {
  auto *A = new (C) PtGuardedByAttr(C, *this, arg);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PtGuardedByAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((pt_guarded_by(" << getArg() << ")))";
    break;
  }
}
}

const char *PtGuardedByAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "pt_guarded_by";
  }
}

PtGuardedVarAttr *PtGuardedVarAttr::clone(ASTContext &C) const {
  auto *A = new (C) PtGuardedVarAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PtGuardedVarAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((pt_guarded_var))";
    break;
  }
  case 1 : {
    OS << " [[clang::pt_guarded_var]]";
    break;
  }
}
}

const char *PtGuardedVarAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "pt_guarded_var";
  case 1:
    return "pt_guarded_var";
  }
}

Ptr32Attr *Ptr32Attr::clone(ASTContext &C) const {
  auto *A = new (C) Ptr32Attr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void Ptr32Attr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __ptr32";
    break;
  }
}
}

const char *Ptr32Attr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__ptr32";
  }
}

Ptr64Attr *Ptr64Attr::clone(ASTContext &C) const {
  auto *A = new (C) Ptr64Attr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void Ptr64Attr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __ptr64";
    break;
  }
}
}

const char *Ptr64Attr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__ptr64";
  }
}

PureAttr *PureAttr::clone(ASTContext &C) const {
  auto *A = new (C) PureAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void PureAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((pure))";
    break;
  }
  case 1 : {
    OS << " [[gnu::pure]]";
    break;
  }
}
}

const char *PureAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "pure";
  case 1:
    return "pure";
  }
}

RISCVInterruptAttr *RISCVInterruptAttr::clone(ASTContext &C) const {
  auto *A = new (C) RISCVInterruptAttr(C, *this, interrupt);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void RISCVInterruptAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((interrupt(\"" << RISCVInterruptAttr::ConvertInterruptTypeToStr(getInterrupt()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::interrupt(\"" << RISCVInterruptAttr::ConvertInterruptTypeToStr(getInterrupt()) << "\")]]";
    break;
  }
}
}

const char *RISCVInterruptAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "interrupt";
  case 1:
    return "interrupt";
  }
}

RegCallAttr *RegCallAttr::clone(ASTContext &C) const {
  auto *A = new (C) RegCallAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void RegCallAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((regcall))";
    break;
  }
  case 1 : {
    OS << " [[gnu::regcall]]";
    break;
  }
  case 2 : {
    OS << " __regcall";
    break;
  }
}
}

const char *RegCallAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "regcall";
  case 1:
    return "regcall";
  case 2:
    return "__regcall";
  }
}

ReinitializesAttr *ReinitializesAttr::clone(ASTContext &C) const {
  auto *A = new (C) ReinitializesAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ReinitializesAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((reinitializes))";
    break;
  }
  case 1 : {
    OS << " [[clang::reinitializes]]";
    break;
  }
}
}

const char *ReinitializesAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "reinitializes";
  case 1:
    return "reinitializes";
  }
}

ReleaseCapabilityAttr *ReleaseCapabilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) ReleaseCapabilityAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ReleaseCapabilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((release_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::release_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " __attribute__((release_shared_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 3 : {
    OS << " [[clang::release_shared_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 4 : {
    OS << " __attribute__((release_generic_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 5 : {
    OS << " [[clang::release_generic_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 6 : {
    OS << " __attribute__((unlock_function(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 7 : {
    OS << " [[clang::unlock_function(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
}
}

const char *ReleaseCapabilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "release_capability";
  case 1:
    return "release_capability";
  case 2:
    return "release_shared_capability";
  case 3:
    return "release_shared_capability";
  case 4:
    return "release_generic_capability";
  case 5:
    return "release_generic_capability";
  case 6:
    return "unlock_function";
  case 7:
    return "unlock_function";
  }
}

ReleaseHandleAttr *ReleaseHandleAttr::clone(ASTContext &C) const {
  auto *A = new (C) ReleaseHandleAttr(C, *this, getHandleType());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ReleaseHandleAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((release_handle(\"" << getHandleType() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::release_handle(\"" << getHandleType() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::release_handle(\"" << getHandleType() << "\")]]";
    break;
  }
}
}

const char *ReleaseHandleAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "release_handle";
  case 1:
    return "release_handle";
  case 2:
    return "release_handle";
  }
}

RenderScriptKernelAttr *RenderScriptKernelAttr::clone(ASTContext &C) const {
  auto *A = new (C) RenderScriptKernelAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void RenderScriptKernelAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((kernel))";
    break;
  }
}
}

const char *RenderScriptKernelAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "kernel";
  }
}

ReqdWorkGroupSizeAttr *ReqdWorkGroupSizeAttr::clone(ASTContext &C) const {
  auto *A = new (C) ReqdWorkGroupSizeAttr(C, *this, xDim, yDim, zDim);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ReqdWorkGroupSizeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((reqd_work_group_size(" << getXDim() << ", " << getYDim() << ", " << getZDim() << ")))";
    break;
  }
}
}

const char *ReqdWorkGroupSizeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "reqd_work_group_size";
  }
}

RequiresCapabilityAttr *RequiresCapabilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) RequiresCapabilityAttr(C, *this, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void RequiresCapabilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((requires_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::requires_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " __attribute__((exclusive_locks_required(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 3 : {
    OS << " [[clang::exclusive_locks_required(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 4 : {
    OS << " __attribute__((requires_shared_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 5 : {
    OS << " [[clang::requires_shared_capability(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 6 : {
    OS << " __attribute__((shared_locks_required(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 7 : {
    OS << " [[clang::shared_locks_required(";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
}
}

const char *RequiresCapabilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "requires_capability";
  case 1:
    return "requires_capability";
  case 2:
    return "exclusive_locks_required";
  case 3:
    return "exclusive_locks_required";
  case 4:
    return "requires_shared_capability";
  case 5:
    return "requires_shared_capability";
  case 6:
    return "shared_locks_required";
  case 7:
    return "shared_locks_required";
  }
}

RestrictAttr *RestrictAttr::clone(ASTContext &C) const {
  auto *A = new (C) RestrictAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void RestrictAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(restrict)";
    break;
  }
  case 1 : {
    OS << " __attribute__((malloc))";
    break;
  }
  case 2 : {
    OS << " [[gnu::malloc]]";
    break;
  }
}
}

const char *RestrictAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "restrict";
  case 1:
    return "malloc";
  case 2:
    return "malloc";
  }
}

ReturnTypestateAttr *ReturnTypestateAttr::clone(ASTContext &C) const {
  auto *A = new (C) ReturnTypestateAttr(C, *this, state);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ReturnTypestateAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((return_typestate(\"" << ReturnTypestateAttr::ConvertConsumedStateToStr(getState()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::return_typestate(\"" << ReturnTypestateAttr::ConvertConsumedStateToStr(getState()) << "\")]]";
    break;
  }
}
}

const char *ReturnTypestateAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "return_typestate";
  case 1:
    return "return_typestate";
  }
}

ReturnsNonNullAttr *ReturnsNonNullAttr::clone(ASTContext &C) const {
  auto *A = new (C) ReturnsNonNullAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ReturnsNonNullAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((returns_nonnull))";
    break;
  }
  case 1 : {
    OS << " [[gnu::returns_nonnull]]";
    break;
  }
}
}

const char *ReturnsNonNullAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "returns_nonnull";
  case 1:
    return "returns_nonnull";
  }
}

ReturnsTwiceAttr *ReturnsTwiceAttr::clone(ASTContext &C) const {
  auto *A = new (C) ReturnsTwiceAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ReturnsTwiceAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((returns_twice))";
    break;
  }
  case 1 : {
    OS << " [[gnu::returns_twice]]";
    break;
  }
}
}

const char *ReturnsTwiceAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "returns_twice";
  case 1:
    return "returns_twice";
  }
}

SPtrAttr *SPtrAttr::clone(ASTContext &C) const {
  auto *A = new (C) SPtrAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SPtrAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __sptr";
    break;
  }
}
}

const char *SPtrAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__sptr";
  }
}

SYCLKernelAttr *SYCLKernelAttr::clone(ASTContext &C) const {
  auto *A = new (C) SYCLKernelAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SYCLKernelAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((sycl_kernel))";
    break;
  }
  case 1 : {
    OS << " [[clang::sycl_kernel]]";
    break;
  }
  case 2 : {
    OS << " [[clang::sycl_kernel]]";
    break;
  }
}
}

const char *SYCLKernelAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "sycl_kernel";
  case 1:
    return "sycl_kernel";
  case 2:
    return "sycl_kernel";
  }
}

ScopedLockableAttr *ScopedLockableAttr::clone(ASTContext &C) const {
  auto *A = new (C) ScopedLockableAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ScopedLockableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((scoped_lockable))";
    break;
  }
  case 1 : {
    OS << " [[clang::scoped_lockable]]";
    break;
  }
}
}

const char *ScopedLockableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "scoped_lockable";
  case 1:
    return "scoped_lockable";
  }
}

SectionAttr *SectionAttr::clone(ASTContext &C) const {
  auto *A = new (C) SectionAttr(C, *this, getName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SectionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((section(\"" << getName() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::section(\"" << getName() << "\")]]";
    break;
  }
  case 2 : {
    OS << " __declspec(allocate(\"" << getName() << "\"))";
    break;
  }
}
}

const char *SectionAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "section";
  case 1:
    return "section";
  case 2:
    return "allocate";
  }
}

SelectAnyAttr *SelectAnyAttr::clone(ASTContext &C) const {
  auto *A = new (C) SelectAnyAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SelectAnyAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(selectany)";
    break;
  }
  case 1 : {
    OS << " __attribute__((selectany))";
    break;
  }
  case 2 : {
    OS << " [[gnu::selectany]]";
    break;
  }
}
}

const char *SelectAnyAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "selectany";
  case 1:
    return "selectany";
  case 2:
    return "selectany";
  }
}

SentinelAttr *SentinelAttr::clone(ASTContext &C) const {
  auto *A = new (C) SentinelAttr(C, *this, sentinel, nullPos);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SentinelAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((sentinel(" << getSentinel() << ", " << getNullPos() << ")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::sentinel(" << getSentinel() << ", " << getNullPos() << ")]]";
    break;
  }
}
}

const char *SentinelAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "sentinel";
  case 1:
    return "sentinel";
  }
}

SetTypestateAttr *SetTypestateAttr::clone(ASTContext &C) const {
  auto *A = new (C) SetTypestateAttr(C, *this, newState);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SetTypestateAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((set_typestate(\"" << SetTypestateAttr::ConvertConsumedStateToStr(getNewState()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::set_typestate(\"" << SetTypestateAttr::ConvertConsumedStateToStr(getNewState()) << "\")]]";
    break;
  }
}
}

const char *SetTypestateAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "set_typestate";
  case 1:
    return "set_typestate";
  }
}

SharedTrylockFunctionAttr *SharedTrylockFunctionAttr::clone(ASTContext &C) const {
  auto *A = new (C) SharedTrylockFunctionAttr(C, *this, successValue, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SharedTrylockFunctionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((shared_trylock_function(" << getSuccessValue() << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
}
}

const char *SharedTrylockFunctionAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "shared_trylock_function";
  }
}

SpeculativeLoadHardeningAttr *SpeculativeLoadHardeningAttr::clone(ASTContext &C) const {
  auto *A = new (C) SpeculativeLoadHardeningAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SpeculativeLoadHardeningAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((speculative_load_hardening))";
    break;
  }
  case 1 : {
    OS << " [[clang::speculative_load_hardening]]";
    break;
  }
  case 2 : {
    OS << " [[clang::speculative_load_hardening]]";
    break;
  }
}
}

const char *SpeculativeLoadHardeningAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "speculative_load_hardening";
  case 1:
    return "speculative_load_hardening";
  case 2:
    return "speculative_load_hardening";
  }
}

StdCallAttr *StdCallAttr::clone(ASTContext &C) const {
  auto *A = new (C) StdCallAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void StdCallAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((stdcall))";
    break;
  }
  case 1 : {
    OS << " [[gnu::stdcall]]";
    break;
  }
  case 2 : {
    OS << " __stdcall";
    break;
  }
  case 3 : {
    OS << " _stdcall";
    break;
  }
}
}

const char *StdCallAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "stdcall";
  case 1:
    return "stdcall";
  case 2:
    return "__stdcall";
  case 3:
    return "_stdcall";
  }
}

SuppressAttr *SuppressAttr::clone(ASTContext &C) const {
  auto *A = new (C) SuppressAttr(C, *this, diagnosticIdentifiers_, diagnosticIdentifiers_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SuppressAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " [[gsl::suppress(";
  bool isFirst = true;
  for (const auto &Val : diagnosticIdentifiers()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << "\"" << Val << "\"";
  }
  OS << ")]]";
    break;
  }
}
}

const char *SuppressAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "suppress";
  }
}

SwiftCallAttr *SwiftCallAttr::clone(ASTContext &C) const {
  auto *A = new (C) SwiftCallAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SwiftCallAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((swiftcall))";
    break;
  }
  case 1 : {
    OS << " [[clang::swiftcall]]";
    break;
  }
  case 2 : {
    OS << " [[clang::swiftcall]]";
    break;
  }
}
}

const char *SwiftCallAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "swiftcall";
  case 1:
    return "swiftcall";
  case 2:
    return "swiftcall";
  }
}

SwiftContextAttr *SwiftContextAttr::clone(ASTContext &C) const {
  auto *A = new (C) SwiftContextAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SwiftContextAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((swift_context))";
    break;
  }
  case 1 : {
    OS << " [[clang::swift_context]]";
    break;
  }
  case 2 : {
    OS << " [[clang::swift_context]]";
    break;
  }
}
}

const char *SwiftContextAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "swift_context";
  case 1:
    return "swift_context";
  case 2:
    return "swift_context";
  }
}

SwiftErrorResultAttr *SwiftErrorResultAttr::clone(ASTContext &C) const {
  auto *A = new (C) SwiftErrorResultAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SwiftErrorResultAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((swift_error_result))";
    break;
  }
  case 1 : {
    OS << " [[clang::swift_error_result]]";
    break;
  }
  case 2 : {
    OS << " [[clang::swift_error_result]]";
    break;
  }
}
}

const char *SwiftErrorResultAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "swift_error_result";
  case 1:
    return "swift_error_result";
  case 2:
    return "swift_error_result";
  }
}

SwiftIndirectResultAttr *SwiftIndirectResultAttr::clone(ASTContext &C) const {
  auto *A = new (C) SwiftIndirectResultAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SwiftIndirectResultAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((swift_indirect_result))";
    break;
  }
  case 1 : {
    OS << " [[clang::swift_indirect_result]]";
    break;
  }
  case 2 : {
    OS << " [[clang::swift_indirect_result]]";
    break;
  }
}
}

const char *SwiftIndirectResultAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "swift_indirect_result";
  case 1:
    return "swift_indirect_result";
  case 2:
    return "swift_indirect_result";
  }
}

SysVABIAttr *SysVABIAttr::clone(ASTContext &C) const {
  auto *A = new (C) SysVABIAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void SysVABIAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((sysv_abi))";
    break;
  }
  case 1 : {
    OS << " [[gnu::sysv_abi]]";
    break;
  }
}
}

const char *SysVABIAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "sysv_abi";
  case 1:
    return "sysv_abi";
  }
}

TLSModelAttr *TLSModelAttr::clone(ASTContext &C) const {
  auto *A = new (C) TLSModelAttr(C, *this, getModel());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TLSModelAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((tls_model(\"" << getModel() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::tls_model(\"" << getModel() << "\")]]";
    break;
  }
}
}

const char *TLSModelAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "tls_model";
  case 1:
    return "tls_model";
  }
}

TargetAttr *TargetAttr::clone(ASTContext &C) const {
  auto *A = new (C) TargetAttr(C, *this, getFeaturesStr());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TargetAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((target(\"" << getFeaturesStr() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::target(\"" << getFeaturesStr() << "\")]]";
    break;
  }
}
}

const char *TargetAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "target";
  case 1:
    return "target";
  }
}

TestTypestateAttr *TestTypestateAttr::clone(ASTContext &C) const {
  auto *A = new (C) TestTypestateAttr(C, *this, testState);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TestTypestateAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((test_typestate(\"" << TestTypestateAttr::ConvertConsumedStateToStr(getTestState()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::test_typestate(\"" << TestTypestateAttr::ConvertConsumedStateToStr(getTestState()) << "\")]]";
    break;
  }
}
}

const char *TestTypestateAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "test_typestate";
  case 1:
    return "test_typestate";
  }
}

ThisCallAttr *ThisCallAttr::clone(ASTContext &C) const {
  auto *A = new (C) ThisCallAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ThisCallAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((thiscall))";
    break;
  }
  case 1 : {
    OS << " [[gnu::thiscall]]";
    break;
  }
  case 2 : {
    OS << " __thiscall";
    break;
  }
  case 3 : {
    OS << " _thiscall";
    break;
  }
}
}

const char *ThisCallAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "thiscall";
  case 1:
    return "thiscall";
  case 2:
    return "__thiscall";
  case 3:
    return "_thiscall";
  }
}

ThreadAttr *ThreadAttr::clone(ASTContext &C) const {
  auto *A = new (C) ThreadAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void ThreadAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(thread)";
    break;
  }
}
}

const char *ThreadAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "thread";
  }
}

TransparentUnionAttr *TransparentUnionAttr::clone(ASTContext &C) const {
  auto *A = new (C) TransparentUnionAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TransparentUnionAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((transparent_union))";
    break;
  }
  case 1 : {
    OS << " [[gnu::transparent_union]]";
    break;
  }
}
}

const char *TransparentUnionAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "transparent_union";
  case 1:
    return "transparent_union";
  }
}

TrivialABIAttr *TrivialABIAttr::clone(ASTContext &C) const {
  auto *A = new (C) TrivialABIAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TrivialABIAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((trivial_abi))";
    break;
  }
  case 1 : {
    OS << " [[clang::trivial_abi]]";
    break;
  }
}
}

const char *TrivialABIAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "trivial_abi";
  case 1:
    return "trivial_abi";
  }
}

TryAcquireCapabilityAttr *TryAcquireCapabilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) TryAcquireCapabilityAttr(C, *this, successValue, args_, args_Size);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TryAcquireCapabilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((try_acquire_capability(" << getSuccessValue() << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::try_acquire_capability(" << getSuccessValue() << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
  case 2 : {
    OS << " __attribute__((try_acquire_shared_capability(" << getSuccessValue() << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")))";
    break;
  }
  case 3 : {
    OS << " [[clang::try_acquire_shared_capability(" << getSuccessValue() << ", ";
  bool isFirst = true;
  for (const auto &Val : args()) {
    if (isFirst) isFirst = false;
    else OS << ", ";
    OS << Val;
  }
  OS << ")]]";
    break;
  }
}
}

const char *TryAcquireCapabilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "try_acquire_capability";
  case 1:
    return "try_acquire_capability";
  case 2:
    return "try_acquire_shared_capability";
  case 3:
    return "try_acquire_shared_capability";
  }
}

TypeNonNullAttr *TypeNonNullAttr::clone(ASTContext &C) const {
  auto *A = new (C) TypeNonNullAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TypeNonNullAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " _Nonnull";
    break;
  }
}
}

const char *TypeNonNullAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "_Nonnull";
  }
}

TypeNullUnspecifiedAttr *TypeNullUnspecifiedAttr::clone(ASTContext &C) const {
  auto *A = new (C) TypeNullUnspecifiedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TypeNullUnspecifiedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " _Null_unspecified";
    break;
  }
}
}

const char *TypeNullUnspecifiedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "_Null_unspecified";
  }
}

TypeNullableAttr *TypeNullableAttr::clone(ASTContext &C) const {
  auto *A = new (C) TypeNullableAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TypeNullableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " _Nullable";
    break;
  }
}
}

const char *TypeNullableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "_Nullable";
  }
}

TypeTagForDatatypeAttr *TypeTagForDatatypeAttr::clone(ASTContext &C) const {
  auto *A = new (C) TypeTagForDatatypeAttr(C, *this, argumentKind, matchingCType, layoutCompatible, mustBeNull);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TypeTagForDatatypeAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((type_tag_for_datatype(" << (getArgumentKind() ? getArgumentKind()->getName() : "") << ", " << getMatchingCType().getAsString() << ", " << getLayoutCompatible() << ", " << getMustBeNull() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::type_tag_for_datatype(" << (getArgumentKind() ? getArgumentKind()->getName() : "") << ", " << getMatchingCType().getAsString() << ", " << getLayoutCompatible() << ", " << getMustBeNull() << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::type_tag_for_datatype(" << (getArgumentKind() ? getArgumentKind()->getName() : "") << ", " << getMatchingCType().getAsString() << ", " << getLayoutCompatible() << ", " << getMustBeNull() << ")]]";
    break;
  }
}
}

const char *TypeTagForDatatypeAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "type_tag_for_datatype";
  case 1:
    return "type_tag_for_datatype";
  case 2:
    return "type_tag_for_datatype";
  }
}

TypeVisibilityAttr *TypeVisibilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) TypeVisibilityAttr(C, *this, visibility);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void TypeVisibilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((type_visibility(\"" << TypeVisibilityAttr::ConvertVisibilityTypeToStr(getVisibility()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::type_visibility(\"" << TypeVisibilityAttr::ConvertVisibilityTypeToStr(getVisibility()) << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::type_visibility(\"" << TypeVisibilityAttr::ConvertVisibilityTypeToStr(getVisibility()) << "\")]]";
    break;
  }
}
}

const char *TypeVisibilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "type_visibility";
  case 1:
    return "type_visibility";
  case 2:
    return "type_visibility";
  }
}

UPtrAttr *UPtrAttr::clone(ASTContext &C) const {
  auto *A = new (C) UPtrAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void UPtrAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __uptr";
    break;
  }
}
}

const char *UPtrAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "__uptr";
  }
}

UnavailableAttr *UnavailableAttr::clone(ASTContext &C) const {
  auto *A = new (C) UnavailableAttr(C, *this, getMessage(), implicitReason);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void UnavailableAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((unavailable(\"" << getMessage() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::unavailable(\"" << getMessage() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::unavailable(\"" << getMessage() << "\")]]";
    break;
  }
}
}

const char *UnavailableAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "unavailable";
  case 1:
    return "unavailable";
  case 2:
    return "unavailable";
  }
}

UninitializedAttr *UninitializedAttr::clone(ASTContext &C) const {
  auto *A = new (C) UninitializedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void UninitializedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((uninitialized))";
    break;
  }
  case 1 : {
    OS << " [[clang::uninitialized]]";
    break;
  }
}
}

const char *UninitializedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "uninitialized";
  case 1:
    return "uninitialized";
  }
}

UnusedAttr *UnusedAttr::clone(ASTContext &C) const {
  auto *A = new (C) UnusedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void UnusedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " [[maybe_unused]]";
    break;
  }
  case 1 : {
    OS << " __attribute__((unused))";
    break;
  }
  case 2 : {
    OS << " [[gnu::unused]]";
    break;
  }
  case 3 : {
    OS << " [[maybe_unused]]";
    break;
  }
}
}

const char *UnusedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "maybe_unused";
  case 1:
    return "unused";
  case 2:
    return "unused";
  case 3:
    return "maybe_unused";
  }
}

UseHandleAttr *UseHandleAttr::clone(ASTContext &C) const {
  auto *A = new (C) UseHandleAttr(C, *this, getHandleType());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void UseHandleAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((use_handle(\"" << getHandleType() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::use_handle(\"" << getHandleType() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::use_handle(\"" << getHandleType() << "\")]]";
    break;
  }
}
}

const char *UseHandleAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "use_handle";
  case 1:
    return "use_handle";
  case 2:
    return "use_handle";
  }
}

UsedAttr *UsedAttr::clone(ASTContext &C) const {
  auto *A = new (C) UsedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void UsedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((used))";
    break;
  }
  case 1 : {
    OS << " [[gnu::used]]";
    break;
  }
}
}

const char *UsedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "used";
  case 1:
    return "used";
  }
}

UuidAttr *UuidAttr::clone(ASTContext &C) const {
  auto *A = new (C) UuidAttr(C, *this, getGuid());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void UuidAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __declspec(uuid(\"" << getGuid() << "\"))";
    break;
  }
  case 1 : {
    OS << "[uuid(\"" << getGuid() << "\")]";
    break;
  }
}
}

const char *UuidAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "uuid";
  case 1:
    return "uuid";
  }
}

VecReturnAttr *VecReturnAttr::clone(ASTContext &C) const {
  auto *A = new (C) VecReturnAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void VecReturnAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((vecreturn))";
    break;
  }
  case 1 : {
    OS << " [[clang::vecreturn]]";
    break;
  }
}
}

const char *VecReturnAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "vecreturn";
  case 1:
    return "vecreturn";
  }
}

VecTypeHintAttr *VecTypeHintAttr::clone(ASTContext &C) const {
  auto *A = new (C) VecTypeHintAttr(C, *this, typeHint);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void VecTypeHintAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((vec_type_hint(" << getTypeHint().getAsString() << ")))";
    break;
  }
}
}

const char *VecTypeHintAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "vec_type_hint";
  }
}

VectorCallAttr *VectorCallAttr::clone(ASTContext &C) const {
  auto *A = new (C) VectorCallAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void VectorCallAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((vectorcall))";
    break;
  }
  case 1 : {
    OS << " [[clang::vectorcall]]";
    break;
  }
  case 2 : {
    OS << " [[clang::vectorcall]]";
    break;
  }
  case 3 : {
    OS << " __vectorcall";
    break;
  }
  case 4 : {
    OS << " _vectorcall";
    break;
  }
}
}

const char *VectorCallAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "vectorcall";
  case 1:
    return "vectorcall";
  case 2:
    return "vectorcall";
  case 3:
    return "__vectorcall";
  case 4:
    return "_vectorcall";
  }
}

VisibilityAttr *VisibilityAttr::clone(ASTContext &C) const {
  auto *A = new (C) VisibilityAttr(C, *this, visibility);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void VisibilityAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((visibility(\"" << VisibilityAttr::ConvertVisibilityTypeToStr(getVisibility()) << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::visibility(\"" << VisibilityAttr::ConvertVisibilityTypeToStr(getVisibility()) << "\")]]";
    break;
  }
}
}

const char *VisibilityAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "visibility";
  case 1:
    return "visibility";
  }
}

WarnUnusedAttr *WarnUnusedAttr::clone(ASTContext &C) const {
  auto *A = new (C) WarnUnusedAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void WarnUnusedAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((warn_unused))";
    break;
  }
  case 1 : {
    OS << " [[gnu::warn_unused]]";
    break;
  }
}
}

const char *WarnUnusedAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "warn_unused";
  case 1:
    return "warn_unused";
  }
}

WarnUnusedResultAttr *WarnUnusedResultAttr::clone(ASTContext &C) const {
  auto *A = new (C) WarnUnusedResultAttr(C, *this, getMessage());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void WarnUnusedResultAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " [[nodiscard(\"" << getMessage() << "\")]]";
    break;
  }
  case 1 : {
    OS << " [[nodiscard(\"" << getMessage() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::warn_unused_result(\"" << getMessage() << "\")]]";
    break;
  }
  case 3 : {
    OS << " __attribute__((warn_unused_result(\"" << getMessage() << "\")))";
    break;
  }
  case 4 : {
    OS << " [[gnu::warn_unused_result(\"" << getMessage() << "\")]]";
    break;
  }
}
}

const char *WarnUnusedResultAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "nodiscard";
  case 1:
    return "nodiscard";
  case 2:
    return "warn_unused_result";
  case 3:
    return "warn_unused_result";
  case 4:
    return "warn_unused_result";
  }
}

WeakAttr *WeakAttr::clone(ASTContext &C) const {
  auto *A = new (C) WeakAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void WeakAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((weak))";
    break;
  }
  case 1 : {
    OS << " [[gnu::weak]]";
    break;
  }
}
}

const char *WeakAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "weak";
  case 1:
    return "weak";
  }
}

WeakImportAttr *WeakImportAttr::clone(ASTContext &C) const {
  auto *A = new (C) WeakImportAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void WeakImportAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((weak_import))";
    break;
  }
  case 1 : {
    OS << " [[clang::weak_import]]";
    break;
  }
  case 2 : {
    OS << " [[clang::weak_import]]";
    break;
  }
}
}

const char *WeakImportAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "weak_import";
  case 1:
    return "weak_import";
  case 2:
    return "weak_import";
  }
}

WeakRefAttr *WeakRefAttr::clone(ASTContext &C) const {
  auto *A = new (C) WeakRefAttr(C, *this, getAliasee());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void WeakRefAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((weakref(\"" << getAliasee() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[gnu::weakref(\"" << getAliasee() << "\")]]";
    break;
  }
}
}

const char *WeakRefAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "weakref";
  case 1:
    return "weakref";
  }
}

WebAssemblyExportNameAttr *WebAssemblyExportNameAttr::clone(ASTContext &C) const {
  auto *A = new (C) WebAssemblyExportNameAttr(C, *this, getExportName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void WebAssemblyExportNameAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((export_name(\"" << getExportName() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::export_name(\"" << getExportName() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::export_name(\"" << getExportName() << "\")]]";
    break;
  }
}
}

const char *WebAssemblyExportNameAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "export_name";
  case 1:
    return "export_name";
  case 2:
    return "export_name";
  }
}

WebAssemblyImportModuleAttr *WebAssemblyImportModuleAttr::clone(ASTContext &C) const {
  auto *A = new (C) WebAssemblyImportModuleAttr(C, *this, getImportModule());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void WebAssemblyImportModuleAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((import_module(\"" << getImportModule() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::import_module(\"" << getImportModule() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::import_module(\"" << getImportModule() << "\")]]";
    break;
  }
}
}

const char *WebAssemblyImportModuleAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "import_module";
  case 1:
    return "import_module";
  case 2:
    return "import_module";
  }
}

WebAssemblyImportNameAttr *WebAssemblyImportNameAttr::clone(ASTContext &C) const {
  auto *A = new (C) WebAssemblyImportNameAttr(C, *this, getImportName());
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void WebAssemblyImportNameAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((import_name(\"" << getImportName() << "\")))";
    break;
  }
  case 1 : {
    OS << " [[clang::import_name(\"" << getImportName() << "\")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::import_name(\"" << getImportName() << "\")]]";
    break;
  }
}
}

const char *WebAssemblyImportNameAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "import_name";
  case 1:
    return "import_name";
  case 2:
    return "import_name";
  }
}

WorkGroupSizeHintAttr *WorkGroupSizeHintAttr::clone(ASTContext &C) const {
  auto *A = new (C) WorkGroupSizeHintAttr(C, *this, xDim, yDim, zDim);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void WorkGroupSizeHintAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((work_group_size_hint(" << getXDim() << ", " << getYDim() << ", " << getZDim() << ")))";
    break;
  }
}
}

const char *WorkGroupSizeHintAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "work_group_size_hint";
  }
}

X86ForceAlignArgPointerAttr *X86ForceAlignArgPointerAttr::clone(ASTContext &C) const {
  auto *A = new (C) X86ForceAlignArgPointerAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void X86ForceAlignArgPointerAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((force_align_arg_pointer))";
    break;
  }
  case 1 : {
    OS << " [[gnu::force_align_arg_pointer]]";
    break;
  }
}
}

const char *X86ForceAlignArgPointerAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "force_align_arg_pointer";
  case 1:
    return "force_align_arg_pointer";
  }
}

XRayInstrumentAttr *XRayInstrumentAttr::clone(ASTContext &C) const {
  auto *A = new (C) XRayInstrumentAttr(C, *this);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void XRayInstrumentAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((xray_always_instrument))";
    break;
  }
  case 1 : {
    OS << " [[clang::xray_always_instrument]]";
    break;
  }
  case 2 : {
    OS << " [[clang::xray_always_instrument]]";
    break;
  }
  case 3 : {
    OS << " __attribute__((xray_never_instrument))";
    break;
  }
  case 4 : {
    OS << " [[clang::xray_never_instrument]]";
    break;
  }
  case 5 : {
    OS << " [[clang::xray_never_instrument]]";
    break;
  }
}
}

const char *XRayInstrumentAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "xray_always_instrument";
  case 1:
    return "xray_always_instrument";
  case 2:
    return "xray_always_instrument";
  case 3:
    return "xray_never_instrument";
  case 4:
    return "xray_never_instrument";
  case 5:
    return "xray_never_instrument";
  }
}

XRayLogArgsAttr *XRayLogArgsAttr::clone(ASTContext &C) const {
  auto *A = new (C) XRayLogArgsAttr(C, *this, argumentCount);
  A->Inherited = Inherited;
  A->IsPackExpansion = IsPackExpansion;
  A->setImplicit(Implicit);
  return A;
}

void XRayLogArgsAttr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    break;
  case 0 : {
    OS << " __attribute__((xray_log_args(" << getArgumentCount() << ")))";
    break;
  }
  case 1 : {
    OS << " [[clang::xray_log_args(" << getArgumentCount() << ")]]";
    break;
  }
  case 2 : {
    OS << " [[clang::xray_log_args(" << getArgumentCount() << ")]]";
    break;
  }
}
}

const char *XRayLogArgsAttr::getSpelling() const {
  switch (getAttributeSpellingListIndex()) {
  default:
    llvm_unreachable("Unknown attribute spelling!");
    return "(No spelling)";
  case 0:
    return "xray_log_args";
  case 1:
    return "xray_log_args";
  case 2:
    return "xray_log_args";
  }
}

const char *Attr::getSpelling() const {
  switch (getKind()) {
  case attr::AArch64VectorPcs:
    return cast<AArch64VectorPcsAttr>(this)->getSpelling();
  case attr::AMDGPUFlatWorkGroupSize:
    return cast<AMDGPUFlatWorkGroupSizeAttr>(this)->getSpelling();
  case attr::AMDGPUNumSGPR:
    return cast<AMDGPUNumSGPRAttr>(this)->getSpelling();
  case attr::AMDGPUNumVGPR:
    return cast<AMDGPUNumVGPRAttr>(this)->getSpelling();
  case attr::AMDGPUWavesPerEU:
    return cast<AMDGPUWavesPerEUAttr>(this)->getSpelling();
  case attr::ARMInterrupt:
    return cast<ARMInterruptAttr>(this)->getSpelling();
  case attr::AVRInterrupt:
    return cast<AVRInterruptAttr>(this)->getSpelling();
  case attr::AVRSignal:
    return cast<AVRSignalAttr>(this)->getSpelling();
  case attr::AbiTag:
    return cast<AbiTagAttr>(this)->getSpelling();
  case attr::AcquireCapability:
    return cast<AcquireCapabilityAttr>(this)->getSpelling();
  case attr::AcquireHandle:
    return cast<AcquireHandleAttr>(this)->getSpelling();
  case attr::AcquiredAfter:
    return cast<AcquiredAfterAttr>(this)->getSpelling();
  case attr::AcquiredBefore:
    return cast<AcquiredBeforeAttr>(this)->getSpelling();
  case attr::AddressSpace:
    return cast<AddressSpaceAttr>(this)->getSpelling();
  case attr::Alias:
    return cast<AliasAttr>(this)->getSpelling();
  case attr::AlignMac68k:
    return cast<AlignMac68kAttr>(this)->getSpelling();
  case attr::AlignValue:
    return cast<AlignValueAttr>(this)->getSpelling();
  case attr::Aligned:
    return cast<AlignedAttr>(this)->getSpelling();
  case attr::AllocAlign:
    return cast<AllocAlignAttr>(this)->getSpelling();
  case attr::AllocSize:
    return cast<AllocSizeAttr>(this)->getSpelling();
  case attr::AlwaysDestroy:
    return cast<AlwaysDestroyAttr>(this)->getSpelling();
  case attr::AlwaysInline:
    return cast<AlwaysInlineAttr>(this)->getSpelling();
  case attr::AnalyzerNoReturn:
    return cast<AnalyzerNoReturnAttr>(this)->getSpelling();
  case attr::Annotate:
    return cast<AnnotateAttr>(this)->getSpelling();
  case attr::AnyX86Interrupt:
    return cast<AnyX86InterruptAttr>(this)->getSpelling();
  case attr::AnyX86NoCallerSavedRegisters:
    return cast<AnyX86NoCallerSavedRegistersAttr>(this)->getSpelling();
  case attr::AnyX86NoCfCheck:
    return cast<AnyX86NoCfCheckAttr>(this)->getSpelling();
  case attr::ArcWeakrefUnavailable:
    return cast<ArcWeakrefUnavailableAttr>(this)->getSpelling();
  case attr::ArgumentWithTypeTag:
    return cast<ArgumentWithTypeTagAttr>(this)->getSpelling();
  case attr::ArmMveAlias:
    return cast<ArmMveAliasAttr>(this)->getSpelling();
  case attr::ArmMveStrictPolymorphism:
    return cast<ArmMveStrictPolymorphismAttr>(this)->getSpelling();
  case attr::Artificial:
    return cast<ArtificialAttr>(this)->getSpelling();
  case attr::AsmLabel:
    return cast<AsmLabelAttr>(this)->getSpelling();
  case attr::AssertCapability:
    return cast<AssertCapabilityAttr>(this)->getSpelling();
  case attr::AssertExclusiveLock:
    return cast<AssertExclusiveLockAttr>(this)->getSpelling();
  case attr::AssertSharedLock:
    return cast<AssertSharedLockAttr>(this)->getSpelling();
  case attr::AssumeAligned:
    return cast<AssumeAlignedAttr>(this)->getSpelling();
  case attr::Availability:
    return cast<AvailabilityAttr>(this)->getSpelling();
  case attr::BPFPreserveAccessIndex:
    return cast<BPFPreserveAccessIndexAttr>(this)->getSpelling();
  case attr::Blocks:
    return cast<BlocksAttr>(this)->getSpelling();
  case attr::C11NoReturn:
    return cast<C11NoReturnAttr>(this)->getSpelling();
  case attr::CDecl:
    return cast<CDeclAttr>(this)->getSpelling();
  case attr::CFAuditedTransfer:
    return cast<CFAuditedTransferAttr>(this)->getSpelling();
  case attr::CFConsumed:
    return cast<CFConsumedAttr>(this)->getSpelling();
  case attr::CFGuard:
    return cast<CFGuardAttr>(this)->getSpelling();
  case attr::CFICanonicalJumpTable:
    return cast<CFICanonicalJumpTableAttr>(this)->getSpelling();
  case attr::CFReturnsNotRetained:
    return cast<CFReturnsNotRetainedAttr>(this)->getSpelling();
  case attr::CFReturnsRetained:
    return cast<CFReturnsRetainedAttr>(this)->getSpelling();
  case attr::CFUnknownTransfer:
    return cast<CFUnknownTransferAttr>(this)->getSpelling();
  case attr::CPUDispatch:
    return cast<CPUDispatchAttr>(this)->getSpelling();
  case attr::CPUSpecific:
    return cast<CPUSpecificAttr>(this)->getSpelling();
  case attr::CUDAConstant:
    return cast<CUDAConstantAttr>(this)->getSpelling();
  case attr::CUDADevice:
    return cast<CUDADeviceAttr>(this)->getSpelling();
  case attr::CUDAGlobal:
    return cast<CUDAGlobalAttr>(this)->getSpelling();
  case attr::CUDAHost:
    return cast<CUDAHostAttr>(this)->getSpelling();
  case attr::CUDAInvalidTarget:
    return cast<CUDAInvalidTargetAttr>(this)->getSpelling();
  case attr::CUDALaunchBounds:
    return cast<CUDALaunchBoundsAttr>(this)->getSpelling();
  case attr::CUDAShared:
    return cast<CUDASharedAttr>(this)->getSpelling();
  case attr::CXX11NoReturn:
    return cast<CXX11NoReturnAttr>(this)->getSpelling();
  case attr::CallableWhen:
    return cast<CallableWhenAttr>(this)->getSpelling();
  case attr::Callback:
    return cast<CallbackAttr>(this)->getSpelling();
  case attr::Capability:
    return cast<CapabilityAttr>(this)->getSpelling();
  case attr::CapturedRecord:
    return cast<CapturedRecordAttr>(this)->getSpelling();
  case attr::CarriesDependency:
    return cast<CarriesDependencyAttr>(this)->getSpelling();
  case attr::Cleanup:
    return cast<CleanupAttr>(this)->getSpelling();
  case attr::CodeSeg:
    return cast<CodeSegAttr>(this)->getSpelling();
  case attr::Cold:
    return cast<ColdAttr>(this)->getSpelling();
  case attr::Common:
    return cast<CommonAttr>(this)->getSpelling();
  case attr::Const:
    return cast<ConstAttr>(this)->getSpelling();
  case attr::ConstInit:
    return cast<ConstInitAttr>(this)->getSpelling();
  case attr::Constructor:
    return cast<ConstructorAttr>(this)->getSpelling();
  case attr::Consumable:
    return cast<ConsumableAttr>(this)->getSpelling();
  case attr::ConsumableAutoCast:
    return cast<ConsumableAutoCastAttr>(this)->getSpelling();
  case attr::ConsumableSetOnRead:
    return cast<ConsumableSetOnReadAttr>(this)->getSpelling();
  case attr::Convergent:
    return cast<ConvergentAttr>(this)->getSpelling();
  case attr::DLLExport:
    return cast<DLLExportAttr>(this)->getSpelling();
  case attr::DLLExportStaticLocal:
    return cast<DLLExportStaticLocalAttr>(this)->getSpelling();
  case attr::DLLImport:
    return cast<DLLImportAttr>(this)->getSpelling();
  case attr::DLLImportStaticLocal:
    return cast<DLLImportStaticLocalAttr>(this)->getSpelling();
  case attr::Deprecated:
    return cast<DeprecatedAttr>(this)->getSpelling();
  case attr::Destructor:
    return cast<DestructorAttr>(this)->getSpelling();
  case attr::DiagnoseIf:
    return cast<DiagnoseIfAttr>(this)->getSpelling();
  case attr::DisableTailCalls:
    return cast<DisableTailCallsAttr>(this)->getSpelling();
  case attr::EmptyBases:
    return cast<EmptyBasesAttr>(this)->getSpelling();
  case attr::EnableIf:
    return cast<EnableIfAttr>(this)->getSpelling();
  case attr::EnumExtensibility:
    return cast<EnumExtensibilityAttr>(this)->getSpelling();
  case attr::ExcludeFromExplicitInstantiation:
    return cast<ExcludeFromExplicitInstantiationAttr>(this)->getSpelling();
  case attr::ExclusiveTrylockFunction:
    return cast<ExclusiveTrylockFunctionAttr>(this)->getSpelling();
  case attr::ExternalSourceSymbol:
    return cast<ExternalSourceSymbolAttr>(this)->getSpelling();
  case attr::FallThrough:
    return cast<FallThroughAttr>(this)->getSpelling();
  case attr::FastCall:
    return cast<FastCallAttr>(this)->getSpelling();
  case attr::Final:
    return cast<FinalAttr>(this)->getSpelling();
  case attr::FlagEnum:
    return cast<FlagEnumAttr>(this)->getSpelling();
  case attr::Flatten:
    return cast<FlattenAttr>(this)->getSpelling();
  case attr::Format:
    return cast<FormatAttr>(this)->getSpelling();
  case attr::FormatArg:
    return cast<FormatArgAttr>(this)->getSpelling();
  case attr::GNUInline:
    return cast<GNUInlineAttr>(this)->getSpelling();
  case attr::GuardedBy:
    return cast<GuardedByAttr>(this)->getSpelling();
  case attr::GuardedVar:
    return cast<GuardedVarAttr>(this)->getSpelling();
  case attr::HIPPinnedShadow:
    return cast<HIPPinnedShadowAttr>(this)->getSpelling();
  case attr::Hot:
    return cast<HotAttr>(this)->getSpelling();
  case attr::IBAction:
    return cast<IBActionAttr>(this)->getSpelling();
  case attr::IBOutlet:
    return cast<IBOutletAttr>(this)->getSpelling();
  case attr::IBOutletCollection:
    return cast<IBOutletCollectionAttr>(this)->getSpelling();
  case attr::IFunc:
    return cast<IFuncAttr>(this)->getSpelling();
  case attr::InitPriority:
    return cast<InitPriorityAttr>(this)->getSpelling();
  case attr::InitSeg:
    return cast<InitSegAttr>(this)->getSpelling();
  case attr::IntelOclBicc:
    return cast<IntelOclBiccAttr>(this)->getSpelling();
  case attr::InternalLinkage:
    return cast<InternalLinkageAttr>(this)->getSpelling();
  case attr::LTOVisibilityPublic:
    return cast<LTOVisibilityPublicAttr>(this)->getSpelling();
  case attr::LayoutVersion:
    return cast<LayoutVersionAttr>(this)->getSpelling();
  case attr::LifetimeBound:
    return cast<LifetimeBoundAttr>(this)->getSpelling();
  case attr::LockReturned:
    return cast<LockReturnedAttr>(this)->getSpelling();
  case attr::LocksExcluded:
    return cast<LocksExcludedAttr>(this)->getSpelling();
  case attr::LoopHint:
    return cast<LoopHintAttr>(this)->getSpelling();
  case attr::MIGServerRoutine:
    return cast<MIGServerRoutineAttr>(this)->getSpelling();
  case attr::MSABI:
    return cast<MSABIAttr>(this)->getSpelling();
  case attr::MSAllocator:
    return cast<MSAllocatorAttr>(this)->getSpelling();
  case attr::MSInheritance:
    return cast<MSInheritanceAttr>(this)->getSpelling();
  case attr::MSNoVTable:
    return cast<MSNoVTableAttr>(this)->getSpelling();
  case attr::MSP430Interrupt:
    return cast<MSP430InterruptAttr>(this)->getSpelling();
  case attr::MSStruct:
    return cast<MSStructAttr>(this)->getSpelling();
  case attr::MSVtorDisp:
    return cast<MSVtorDispAttr>(this)->getSpelling();
  case attr::MaxFieldAlignment:
    return cast<MaxFieldAlignmentAttr>(this)->getSpelling();
  case attr::MayAlias:
    return cast<MayAliasAttr>(this)->getSpelling();
  case attr::MicroMips:
    return cast<MicroMipsAttr>(this)->getSpelling();
  case attr::MinSize:
    return cast<MinSizeAttr>(this)->getSpelling();
  case attr::MinVectorWidth:
    return cast<MinVectorWidthAttr>(this)->getSpelling();
  case attr::Mips16:
    return cast<Mips16Attr>(this)->getSpelling();
  case attr::MipsInterrupt:
    return cast<MipsInterruptAttr>(this)->getSpelling();
  case attr::MipsLongCall:
    return cast<MipsLongCallAttr>(this)->getSpelling();
  case attr::MipsShortCall:
    return cast<MipsShortCallAttr>(this)->getSpelling();
  case attr::Mode:
    return cast<ModeAttr>(this)->getSpelling();
  case attr::NSConsumed:
    return cast<NSConsumedAttr>(this)->getSpelling();
  case attr::NSConsumesSelf:
    return cast<NSConsumesSelfAttr>(this)->getSpelling();
  case attr::NSReturnsAutoreleased:
    return cast<NSReturnsAutoreleasedAttr>(this)->getSpelling();
  case attr::NSReturnsNotRetained:
    return cast<NSReturnsNotRetainedAttr>(this)->getSpelling();
  case attr::NSReturnsRetained:
    return cast<NSReturnsRetainedAttr>(this)->getSpelling();
  case attr::Naked:
    return cast<NakedAttr>(this)->getSpelling();
  case attr::NoAlias:
    return cast<NoAliasAttr>(this)->getSpelling();
  case attr::NoBuiltin:
    return cast<NoBuiltinAttr>(this)->getSpelling();
  case attr::NoCommon:
    return cast<NoCommonAttr>(this)->getSpelling();
  case attr::NoDebug:
    return cast<NoDebugAttr>(this)->getSpelling();
  case attr::NoDeref:
    return cast<NoDerefAttr>(this)->getSpelling();
  case attr::NoDestroy:
    return cast<NoDestroyAttr>(this)->getSpelling();
  case attr::NoDuplicate:
    return cast<NoDuplicateAttr>(this)->getSpelling();
  case attr::NoEscape:
    return cast<NoEscapeAttr>(this)->getSpelling();
  case attr::NoInline:
    return cast<NoInlineAttr>(this)->getSpelling();
  case attr::NoInstrumentFunction:
    return cast<NoInstrumentFunctionAttr>(this)->getSpelling();
  case attr::NoMicroMips:
    return cast<NoMicroMipsAttr>(this)->getSpelling();
  case attr::NoMips16:
    return cast<NoMips16Attr>(this)->getSpelling();
  case attr::NoReturn:
    return cast<NoReturnAttr>(this)->getSpelling();
  case attr::NoSanitize:
    return cast<NoSanitizeAttr>(this)->getSpelling();
  case attr::NoSpeculativeLoadHardening:
    return cast<NoSpeculativeLoadHardeningAttr>(this)->getSpelling();
  case attr::NoSplitStack:
    return cast<NoSplitStackAttr>(this)->getSpelling();
  case attr::NoStackProtector:
    return cast<NoStackProtectorAttr>(this)->getSpelling();
  case attr::NoThreadSafetyAnalysis:
    return cast<NoThreadSafetyAnalysisAttr>(this)->getSpelling();
  case attr::NoThrow:
    return cast<NoThrowAttr>(this)->getSpelling();
  case attr::NoUniqueAddress:
    return cast<NoUniqueAddressAttr>(this)->getSpelling();
  case attr::NonNull:
    return cast<NonNullAttr>(this)->getSpelling();
  case attr::NotTailCalled:
    return cast<NotTailCalledAttr>(this)->getSpelling();
  case attr::OMPAllocateDecl:
    return cast<OMPAllocateDeclAttr>(this)->getSpelling();
  case attr::OMPCaptureKind:
    return cast<OMPCaptureKindAttr>(this)->getSpelling();
  case attr::OMPCaptureNoInit:
    return cast<OMPCaptureNoInitAttr>(this)->getSpelling();
  case attr::OMPDeclareSimdDecl:
    return cast<OMPDeclareSimdDeclAttr>(this)->getSpelling();
  case attr::OMPDeclareTargetDecl:
    return cast<OMPDeclareTargetDeclAttr>(this)->getSpelling();
  case attr::OMPDeclareVariant:
    return cast<OMPDeclareVariantAttr>(this)->getSpelling();
  case attr::OMPReferencedVar:
    return cast<OMPReferencedVarAttr>(this)->getSpelling();
  case attr::OMPThreadPrivateDecl:
    return cast<OMPThreadPrivateDeclAttr>(this)->getSpelling();
  case attr::OSConsumed:
    return cast<OSConsumedAttr>(this)->getSpelling();
  case attr::OSConsumesThis:
    return cast<OSConsumesThisAttr>(this)->getSpelling();
  case attr::OSReturnsNotRetained:
    return cast<OSReturnsNotRetainedAttr>(this)->getSpelling();
  case attr::OSReturnsRetained:
    return cast<OSReturnsRetainedAttr>(this)->getSpelling();
  case attr::OSReturnsRetainedOnNonZero:
    return cast<OSReturnsRetainedOnNonZeroAttr>(this)->getSpelling();
  case attr::OSReturnsRetainedOnZero:
    return cast<OSReturnsRetainedOnZeroAttr>(this)->getSpelling();
  case attr::ObjCBoxable:
    return cast<ObjCBoxableAttr>(this)->getSpelling();
  case attr::ObjCBridge:
    return cast<ObjCBridgeAttr>(this)->getSpelling();
  case attr::ObjCBridgeMutable:
    return cast<ObjCBridgeMutableAttr>(this)->getSpelling();
  case attr::ObjCBridgeRelated:
    return cast<ObjCBridgeRelatedAttr>(this)->getSpelling();
  case attr::ObjCClassStub:
    return cast<ObjCClassStubAttr>(this)->getSpelling();
  case attr::ObjCDesignatedInitializer:
    return cast<ObjCDesignatedInitializerAttr>(this)->getSpelling();
  case attr::ObjCDirect:
    return cast<ObjCDirectAttr>(this)->getSpelling();
  case attr::ObjCDirectMembers:
    return cast<ObjCDirectMembersAttr>(this)->getSpelling();
  case attr::ObjCException:
    return cast<ObjCExceptionAttr>(this)->getSpelling();
  case attr::ObjCExplicitProtocolImpl:
    return cast<ObjCExplicitProtocolImplAttr>(this)->getSpelling();
  case attr::ObjCExternallyRetained:
    return cast<ObjCExternallyRetainedAttr>(this)->getSpelling();
  case attr::ObjCGC:
    return cast<ObjCGCAttr>(this)->getSpelling();
  case attr::ObjCIndependentClass:
    return cast<ObjCIndependentClassAttr>(this)->getSpelling();
  case attr::ObjCInertUnsafeUnretained:
    return cast<ObjCInertUnsafeUnretainedAttr>(this)->getSpelling();
  case attr::ObjCKindOf:
    return cast<ObjCKindOfAttr>(this)->getSpelling();
  case attr::ObjCMethodFamily:
    return cast<ObjCMethodFamilyAttr>(this)->getSpelling();
  case attr::ObjCNSObject:
    return cast<ObjCNSObjectAttr>(this)->getSpelling();
  case attr::ObjCNonLazyClass:
    return cast<ObjCNonLazyClassAttr>(this)->getSpelling();
  case attr::ObjCOwnership:
    return cast<ObjCOwnershipAttr>(this)->getSpelling();
  case attr::ObjCPreciseLifetime:
    return cast<ObjCPreciseLifetimeAttr>(this)->getSpelling();
  case attr::ObjCRequiresPropertyDefs:
    return cast<ObjCRequiresPropertyDefsAttr>(this)->getSpelling();
  case attr::ObjCRequiresSuper:
    return cast<ObjCRequiresSuperAttr>(this)->getSpelling();
  case attr::ObjCReturnsInnerPointer:
    return cast<ObjCReturnsInnerPointerAttr>(this)->getSpelling();
  case attr::ObjCRootClass:
    return cast<ObjCRootClassAttr>(this)->getSpelling();
  case attr::ObjCRuntimeName:
    return cast<ObjCRuntimeNameAttr>(this)->getSpelling();
  case attr::ObjCRuntimeVisible:
    return cast<ObjCRuntimeVisibleAttr>(this)->getSpelling();
  case attr::ObjCSubclassingRestricted:
    return cast<ObjCSubclassingRestrictedAttr>(this)->getSpelling();
  case attr::OpenCLAccess:
    return cast<OpenCLAccessAttr>(this)->getSpelling();
  case attr::OpenCLConstantAddressSpace:
    return cast<OpenCLConstantAddressSpaceAttr>(this)->getSpelling();
  case attr::OpenCLGenericAddressSpace:
    return cast<OpenCLGenericAddressSpaceAttr>(this)->getSpelling();
  case attr::OpenCLGlobalAddressSpace:
    return cast<OpenCLGlobalAddressSpaceAttr>(this)->getSpelling();
  case attr::OpenCLIntelReqdSubGroupSize:
    return cast<OpenCLIntelReqdSubGroupSizeAttr>(this)->getSpelling();
  case attr::OpenCLKernel:
    return cast<OpenCLKernelAttr>(this)->getSpelling();
  case attr::OpenCLLocalAddressSpace:
    return cast<OpenCLLocalAddressSpaceAttr>(this)->getSpelling();
  case attr::OpenCLPrivateAddressSpace:
    return cast<OpenCLPrivateAddressSpaceAttr>(this)->getSpelling();
  case attr::OpenCLUnrollHint:
    return cast<OpenCLUnrollHintAttr>(this)->getSpelling();
  case attr::OptimizeNone:
    return cast<OptimizeNoneAttr>(this)->getSpelling();
  case attr::Overloadable:
    return cast<OverloadableAttr>(this)->getSpelling();
  case attr::Override:
    return cast<OverrideAttr>(this)->getSpelling();
  case attr::Owner:
    return cast<OwnerAttr>(this)->getSpelling();
  case attr::Ownership:
    return cast<OwnershipAttr>(this)->getSpelling();
  case attr::Packed:
    return cast<PackedAttr>(this)->getSpelling();
  case attr::ParamTypestate:
    return cast<ParamTypestateAttr>(this)->getSpelling();
  case attr::Pascal:
    return cast<PascalAttr>(this)->getSpelling();
  case attr::PassObjectSize:
    return cast<PassObjectSizeAttr>(this)->getSpelling();
  case attr::PatchableFunctionEntry:
    return cast<PatchableFunctionEntryAttr>(this)->getSpelling();
  case attr::Pcs:
    return cast<PcsAttr>(this)->getSpelling();
  case attr::Pointer:
    return cast<PointerAttr>(this)->getSpelling();
  case attr::PragmaClangBSSSection:
    return cast<PragmaClangBSSSectionAttr>(this)->getSpelling();
  case attr::PragmaClangDataSection:
    return cast<PragmaClangDataSectionAttr>(this)->getSpelling();
  case attr::PragmaClangRelroSection:
    return cast<PragmaClangRelroSectionAttr>(this)->getSpelling();
  case attr::PragmaClangRodataSection:
    return cast<PragmaClangRodataSectionAttr>(this)->getSpelling();
  case attr::PragmaClangTextSection:
    return cast<PragmaClangTextSectionAttr>(this)->getSpelling();
  case attr::PreserveAll:
    return cast<PreserveAllAttr>(this)->getSpelling();
  case attr::PreserveMost:
    return cast<PreserveMostAttr>(this)->getSpelling();
  case attr::PtGuardedBy:
    return cast<PtGuardedByAttr>(this)->getSpelling();
  case attr::PtGuardedVar:
    return cast<PtGuardedVarAttr>(this)->getSpelling();
  case attr::Ptr32:
    return cast<Ptr32Attr>(this)->getSpelling();
  case attr::Ptr64:
    return cast<Ptr64Attr>(this)->getSpelling();
  case attr::Pure:
    return cast<PureAttr>(this)->getSpelling();
  case attr::RISCVInterrupt:
    return cast<RISCVInterruptAttr>(this)->getSpelling();
  case attr::RegCall:
    return cast<RegCallAttr>(this)->getSpelling();
  case attr::Reinitializes:
    return cast<ReinitializesAttr>(this)->getSpelling();
  case attr::ReleaseCapability:
    return cast<ReleaseCapabilityAttr>(this)->getSpelling();
  case attr::ReleaseHandle:
    return cast<ReleaseHandleAttr>(this)->getSpelling();
  case attr::RenderScriptKernel:
    return cast<RenderScriptKernelAttr>(this)->getSpelling();
  case attr::ReqdWorkGroupSize:
    return cast<ReqdWorkGroupSizeAttr>(this)->getSpelling();
  case attr::RequiresCapability:
    return cast<RequiresCapabilityAttr>(this)->getSpelling();
  case attr::Restrict:
    return cast<RestrictAttr>(this)->getSpelling();
  case attr::ReturnTypestate:
    return cast<ReturnTypestateAttr>(this)->getSpelling();
  case attr::ReturnsNonNull:
    return cast<ReturnsNonNullAttr>(this)->getSpelling();
  case attr::ReturnsTwice:
    return cast<ReturnsTwiceAttr>(this)->getSpelling();
  case attr::SPtr:
    return cast<SPtrAttr>(this)->getSpelling();
  case attr::SYCLKernel:
    return cast<SYCLKernelAttr>(this)->getSpelling();
  case attr::ScopedLockable:
    return cast<ScopedLockableAttr>(this)->getSpelling();
  case attr::Section:
    return cast<SectionAttr>(this)->getSpelling();
  case attr::SelectAny:
    return cast<SelectAnyAttr>(this)->getSpelling();
  case attr::Sentinel:
    return cast<SentinelAttr>(this)->getSpelling();
  case attr::SetTypestate:
    return cast<SetTypestateAttr>(this)->getSpelling();
  case attr::SharedTrylockFunction:
    return cast<SharedTrylockFunctionAttr>(this)->getSpelling();
  case attr::SpeculativeLoadHardening:
    return cast<SpeculativeLoadHardeningAttr>(this)->getSpelling();
  case attr::StdCall:
    return cast<StdCallAttr>(this)->getSpelling();
  case attr::Suppress:
    return cast<SuppressAttr>(this)->getSpelling();
  case attr::SwiftCall:
    return cast<SwiftCallAttr>(this)->getSpelling();
  case attr::SwiftContext:
    return cast<SwiftContextAttr>(this)->getSpelling();
  case attr::SwiftErrorResult:
    return cast<SwiftErrorResultAttr>(this)->getSpelling();
  case attr::SwiftIndirectResult:
    return cast<SwiftIndirectResultAttr>(this)->getSpelling();
  case attr::SysVABI:
    return cast<SysVABIAttr>(this)->getSpelling();
  case attr::TLSModel:
    return cast<TLSModelAttr>(this)->getSpelling();
  case attr::Target:
    return cast<TargetAttr>(this)->getSpelling();
  case attr::TestTypestate:
    return cast<TestTypestateAttr>(this)->getSpelling();
  case attr::ThisCall:
    return cast<ThisCallAttr>(this)->getSpelling();
  case attr::Thread:
    return cast<ThreadAttr>(this)->getSpelling();
  case attr::TransparentUnion:
    return cast<TransparentUnionAttr>(this)->getSpelling();
  case attr::TrivialABI:
    return cast<TrivialABIAttr>(this)->getSpelling();
  case attr::TryAcquireCapability:
    return cast<TryAcquireCapabilityAttr>(this)->getSpelling();
  case attr::TypeNonNull:
    return cast<TypeNonNullAttr>(this)->getSpelling();
  case attr::TypeNullUnspecified:
    return cast<TypeNullUnspecifiedAttr>(this)->getSpelling();
  case attr::TypeNullable:
    return cast<TypeNullableAttr>(this)->getSpelling();
  case attr::TypeTagForDatatype:
    return cast<TypeTagForDatatypeAttr>(this)->getSpelling();
  case attr::TypeVisibility:
    return cast<TypeVisibilityAttr>(this)->getSpelling();
  case attr::UPtr:
    return cast<UPtrAttr>(this)->getSpelling();
  case attr::Unavailable:
    return cast<UnavailableAttr>(this)->getSpelling();
  case attr::Uninitialized:
    return cast<UninitializedAttr>(this)->getSpelling();
  case attr::Unused:
    return cast<UnusedAttr>(this)->getSpelling();
  case attr::UseHandle:
    return cast<UseHandleAttr>(this)->getSpelling();
  case attr::Used:
    return cast<UsedAttr>(this)->getSpelling();
  case attr::Uuid:
    return cast<UuidAttr>(this)->getSpelling();
  case attr::VecReturn:
    return cast<VecReturnAttr>(this)->getSpelling();
  case attr::VecTypeHint:
    return cast<VecTypeHintAttr>(this)->getSpelling();
  case attr::VectorCall:
    return cast<VectorCallAttr>(this)->getSpelling();
  case attr::Visibility:
    return cast<VisibilityAttr>(this)->getSpelling();
  case attr::WarnUnused:
    return cast<WarnUnusedAttr>(this)->getSpelling();
  case attr::WarnUnusedResult:
    return cast<WarnUnusedResultAttr>(this)->getSpelling();
  case attr::Weak:
    return cast<WeakAttr>(this)->getSpelling();
  case attr::WeakImport:
    return cast<WeakImportAttr>(this)->getSpelling();
  case attr::WeakRef:
    return cast<WeakRefAttr>(this)->getSpelling();
  case attr::WebAssemblyExportName:
    return cast<WebAssemblyExportNameAttr>(this)->getSpelling();
  case attr::WebAssemblyImportModule:
    return cast<WebAssemblyImportModuleAttr>(this)->getSpelling();
  case attr::WebAssemblyImportName:
    return cast<WebAssemblyImportNameAttr>(this)->getSpelling();
  case attr::WorkGroupSizeHint:
    return cast<WorkGroupSizeHintAttr>(this)->getSpelling();
  case attr::X86ForceAlignArgPointer:
    return cast<X86ForceAlignArgPointerAttr>(this)->getSpelling();
  case attr::XRayInstrument:
    return cast<XRayInstrumentAttr>(this)->getSpelling();
  case attr::XRayLogArgs:
    return cast<XRayLogArgsAttr>(this)->getSpelling();
  }
  llvm_unreachable("Unexpected attribute kind!");
}

Attr *Attr::clone(ASTContext &C) const {
  switch (getKind()) {
  case attr::AArch64VectorPcs:
    return cast<AArch64VectorPcsAttr>(this)->clone(C);
  case attr::AMDGPUFlatWorkGroupSize:
    return cast<AMDGPUFlatWorkGroupSizeAttr>(this)->clone(C);
  case attr::AMDGPUNumSGPR:
    return cast<AMDGPUNumSGPRAttr>(this)->clone(C);
  case attr::AMDGPUNumVGPR:
    return cast<AMDGPUNumVGPRAttr>(this)->clone(C);
  case attr::AMDGPUWavesPerEU:
    return cast<AMDGPUWavesPerEUAttr>(this)->clone(C);
  case attr::ARMInterrupt:
    return cast<ARMInterruptAttr>(this)->clone(C);
  case attr::AVRInterrupt:
    return cast<AVRInterruptAttr>(this)->clone(C);
  case attr::AVRSignal:
    return cast<AVRSignalAttr>(this)->clone(C);
  case attr::AbiTag:
    return cast<AbiTagAttr>(this)->clone(C);
  case attr::AcquireCapability:
    return cast<AcquireCapabilityAttr>(this)->clone(C);
  case attr::AcquireHandle:
    return cast<AcquireHandleAttr>(this)->clone(C);
  case attr::AcquiredAfter:
    return cast<AcquiredAfterAttr>(this)->clone(C);
  case attr::AcquiredBefore:
    return cast<AcquiredBeforeAttr>(this)->clone(C);
  case attr::AddressSpace:
    return cast<AddressSpaceAttr>(this)->clone(C);
  case attr::Alias:
    return cast<AliasAttr>(this)->clone(C);
  case attr::AlignMac68k:
    return cast<AlignMac68kAttr>(this)->clone(C);
  case attr::AlignValue:
    return cast<AlignValueAttr>(this)->clone(C);
  case attr::Aligned:
    return cast<AlignedAttr>(this)->clone(C);
  case attr::AllocAlign:
    return cast<AllocAlignAttr>(this)->clone(C);
  case attr::AllocSize:
    return cast<AllocSizeAttr>(this)->clone(C);
  case attr::AlwaysDestroy:
    return cast<AlwaysDestroyAttr>(this)->clone(C);
  case attr::AlwaysInline:
    return cast<AlwaysInlineAttr>(this)->clone(C);
  case attr::AnalyzerNoReturn:
    return cast<AnalyzerNoReturnAttr>(this)->clone(C);
  case attr::Annotate:
    return cast<AnnotateAttr>(this)->clone(C);
  case attr::AnyX86Interrupt:
    return cast<AnyX86InterruptAttr>(this)->clone(C);
  case attr::AnyX86NoCallerSavedRegisters:
    return cast<AnyX86NoCallerSavedRegistersAttr>(this)->clone(C);
  case attr::AnyX86NoCfCheck:
    return cast<AnyX86NoCfCheckAttr>(this)->clone(C);
  case attr::ArcWeakrefUnavailable:
    return cast<ArcWeakrefUnavailableAttr>(this)->clone(C);
  case attr::ArgumentWithTypeTag:
    return cast<ArgumentWithTypeTagAttr>(this)->clone(C);
  case attr::ArmMveAlias:
    return cast<ArmMveAliasAttr>(this)->clone(C);
  case attr::ArmMveStrictPolymorphism:
    return cast<ArmMveStrictPolymorphismAttr>(this)->clone(C);
  case attr::Artificial:
    return cast<ArtificialAttr>(this)->clone(C);
  case attr::AsmLabel:
    return cast<AsmLabelAttr>(this)->clone(C);
  case attr::AssertCapability:
    return cast<AssertCapabilityAttr>(this)->clone(C);
  case attr::AssertExclusiveLock:
    return cast<AssertExclusiveLockAttr>(this)->clone(C);
  case attr::AssertSharedLock:
    return cast<AssertSharedLockAttr>(this)->clone(C);
  case attr::AssumeAligned:
    return cast<AssumeAlignedAttr>(this)->clone(C);
  case attr::Availability:
    return cast<AvailabilityAttr>(this)->clone(C);
  case attr::BPFPreserveAccessIndex:
    return cast<BPFPreserveAccessIndexAttr>(this)->clone(C);
  case attr::Blocks:
    return cast<BlocksAttr>(this)->clone(C);
  case attr::C11NoReturn:
    return cast<C11NoReturnAttr>(this)->clone(C);
  case attr::CDecl:
    return cast<CDeclAttr>(this)->clone(C);
  case attr::CFAuditedTransfer:
    return cast<CFAuditedTransferAttr>(this)->clone(C);
  case attr::CFConsumed:
    return cast<CFConsumedAttr>(this)->clone(C);
  case attr::CFGuard:
    return cast<CFGuardAttr>(this)->clone(C);
  case attr::CFICanonicalJumpTable:
    return cast<CFICanonicalJumpTableAttr>(this)->clone(C);
  case attr::CFReturnsNotRetained:
    return cast<CFReturnsNotRetainedAttr>(this)->clone(C);
  case attr::CFReturnsRetained:
    return cast<CFReturnsRetainedAttr>(this)->clone(C);
  case attr::CFUnknownTransfer:
    return cast<CFUnknownTransferAttr>(this)->clone(C);
  case attr::CPUDispatch:
    return cast<CPUDispatchAttr>(this)->clone(C);
  case attr::CPUSpecific:
    return cast<CPUSpecificAttr>(this)->clone(C);
  case attr::CUDAConstant:
    return cast<CUDAConstantAttr>(this)->clone(C);
  case attr::CUDADevice:
    return cast<CUDADeviceAttr>(this)->clone(C);
  case attr::CUDAGlobal:
    return cast<CUDAGlobalAttr>(this)->clone(C);
  case attr::CUDAHost:
    return cast<CUDAHostAttr>(this)->clone(C);
  case attr::CUDAInvalidTarget:
    return cast<CUDAInvalidTargetAttr>(this)->clone(C);
  case attr::CUDALaunchBounds:
    return cast<CUDALaunchBoundsAttr>(this)->clone(C);
  case attr::CUDAShared:
    return cast<CUDASharedAttr>(this)->clone(C);
  case attr::CXX11NoReturn:
    return cast<CXX11NoReturnAttr>(this)->clone(C);
  case attr::CallableWhen:
    return cast<CallableWhenAttr>(this)->clone(C);
  case attr::Callback:
    return cast<CallbackAttr>(this)->clone(C);
  case attr::Capability:
    return cast<CapabilityAttr>(this)->clone(C);
  case attr::CapturedRecord:
    return cast<CapturedRecordAttr>(this)->clone(C);
  case attr::CarriesDependency:
    return cast<CarriesDependencyAttr>(this)->clone(C);
  case attr::Cleanup:
    return cast<CleanupAttr>(this)->clone(C);
  case attr::CodeSeg:
    return cast<CodeSegAttr>(this)->clone(C);
  case attr::Cold:
    return cast<ColdAttr>(this)->clone(C);
  case attr::Common:
    return cast<CommonAttr>(this)->clone(C);
  case attr::Const:
    return cast<ConstAttr>(this)->clone(C);
  case attr::ConstInit:
    return cast<ConstInitAttr>(this)->clone(C);
  case attr::Constructor:
    return cast<ConstructorAttr>(this)->clone(C);
  case attr::Consumable:
    return cast<ConsumableAttr>(this)->clone(C);
  case attr::ConsumableAutoCast:
    return cast<ConsumableAutoCastAttr>(this)->clone(C);
  case attr::ConsumableSetOnRead:
    return cast<ConsumableSetOnReadAttr>(this)->clone(C);
  case attr::Convergent:
    return cast<ConvergentAttr>(this)->clone(C);
  case attr::DLLExport:
    return cast<DLLExportAttr>(this)->clone(C);
  case attr::DLLExportStaticLocal:
    return cast<DLLExportStaticLocalAttr>(this)->clone(C);
  case attr::DLLImport:
    return cast<DLLImportAttr>(this)->clone(C);
  case attr::DLLImportStaticLocal:
    return cast<DLLImportStaticLocalAttr>(this)->clone(C);
  case attr::Deprecated:
    return cast<DeprecatedAttr>(this)->clone(C);
  case attr::Destructor:
    return cast<DestructorAttr>(this)->clone(C);
  case attr::DiagnoseIf:
    return cast<DiagnoseIfAttr>(this)->clone(C);
  case attr::DisableTailCalls:
    return cast<DisableTailCallsAttr>(this)->clone(C);
  case attr::EmptyBases:
    return cast<EmptyBasesAttr>(this)->clone(C);
  case attr::EnableIf:
    return cast<EnableIfAttr>(this)->clone(C);
  case attr::EnumExtensibility:
    return cast<EnumExtensibilityAttr>(this)->clone(C);
  case attr::ExcludeFromExplicitInstantiation:
    return cast<ExcludeFromExplicitInstantiationAttr>(this)->clone(C);
  case attr::ExclusiveTrylockFunction:
    return cast<ExclusiveTrylockFunctionAttr>(this)->clone(C);
  case attr::ExternalSourceSymbol:
    return cast<ExternalSourceSymbolAttr>(this)->clone(C);
  case attr::FallThrough:
    return cast<FallThroughAttr>(this)->clone(C);
  case attr::FastCall:
    return cast<FastCallAttr>(this)->clone(C);
  case attr::Final:
    return cast<FinalAttr>(this)->clone(C);
  case attr::FlagEnum:
    return cast<FlagEnumAttr>(this)->clone(C);
  case attr::Flatten:
    return cast<FlattenAttr>(this)->clone(C);
  case attr::Format:
    return cast<FormatAttr>(this)->clone(C);
  case attr::FormatArg:
    return cast<FormatArgAttr>(this)->clone(C);
  case attr::GNUInline:
    return cast<GNUInlineAttr>(this)->clone(C);
  case attr::GuardedBy:
    return cast<GuardedByAttr>(this)->clone(C);
  case attr::GuardedVar:
    return cast<GuardedVarAttr>(this)->clone(C);
  case attr::HIPPinnedShadow:
    return cast<HIPPinnedShadowAttr>(this)->clone(C);
  case attr::Hot:
    return cast<HotAttr>(this)->clone(C);
  case attr::IBAction:
    return cast<IBActionAttr>(this)->clone(C);
  case attr::IBOutlet:
    return cast<IBOutletAttr>(this)->clone(C);
  case attr::IBOutletCollection:
    return cast<IBOutletCollectionAttr>(this)->clone(C);
  case attr::IFunc:
    return cast<IFuncAttr>(this)->clone(C);
  case attr::InitPriority:
    return cast<InitPriorityAttr>(this)->clone(C);
  case attr::InitSeg:
    return cast<InitSegAttr>(this)->clone(C);
  case attr::IntelOclBicc:
    return cast<IntelOclBiccAttr>(this)->clone(C);
  case attr::InternalLinkage:
    return cast<InternalLinkageAttr>(this)->clone(C);
  case attr::LTOVisibilityPublic:
    return cast<LTOVisibilityPublicAttr>(this)->clone(C);
  case attr::LayoutVersion:
    return cast<LayoutVersionAttr>(this)->clone(C);
  case attr::LifetimeBound:
    return cast<LifetimeBoundAttr>(this)->clone(C);
  case attr::LockReturned:
    return cast<LockReturnedAttr>(this)->clone(C);
  case attr::LocksExcluded:
    return cast<LocksExcludedAttr>(this)->clone(C);
  case attr::LoopHint:
    return cast<LoopHintAttr>(this)->clone(C);
  case attr::MIGServerRoutine:
    return cast<MIGServerRoutineAttr>(this)->clone(C);
  case attr::MSABI:
    return cast<MSABIAttr>(this)->clone(C);
  case attr::MSAllocator:
    return cast<MSAllocatorAttr>(this)->clone(C);
  case attr::MSInheritance:
    return cast<MSInheritanceAttr>(this)->clone(C);
  case attr::MSNoVTable:
    return cast<MSNoVTableAttr>(this)->clone(C);
  case attr::MSP430Interrupt:
    return cast<MSP430InterruptAttr>(this)->clone(C);
  case attr::MSStruct:
    return cast<MSStructAttr>(this)->clone(C);
  case attr::MSVtorDisp:
    return cast<MSVtorDispAttr>(this)->clone(C);
  case attr::MaxFieldAlignment:
    return cast<MaxFieldAlignmentAttr>(this)->clone(C);
  case attr::MayAlias:
    return cast<MayAliasAttr>(this)->clone(C);
  case attr::MicroMips:
    return cast<MicroMipsAttr>(this)->clone(C);
  case attr::MinSize:
    return cast<MinSizeAttr>(this)->clone(C);
  case attr::MinVectorWidth:
    return cast<MinVectorWidthAttr>(this)->clone(C);
  case attr::Mips16:
    return cast<Mips16Attr>(this)->clone(C);
  case attr::MipsInterrupt:
    return cast<MipsInterruptAttr>(this)->clone(C);
  case attr::MipsLongCall:
    return cast<MipsLongCallAttr>(this)->clone(C);
  case attr::MipsShortCall:
    return cast<MipsShortCallAttr>(this)->clone(C);
  case attr::Mode:
    return cast<ModeAttr>(this)->clone(C);
  case attr::NSConsumed:
    return cast<NSConsumedAttr>(this)->clone(C);
  case attr::NSConsumesSelf:
    return cast<NSConsumesSelfAttr>(this)->clone(C);
  case attr::NSReturnsAutoreleased:
    return cast<NSReturnsAutoreleasedAttr>(this)->clone(C);
  case attr::NSReturnsNotRetained:
    return cast<NSReturnsNotRetainedAttr>(this)->clone(C);
  case attr::NSReturnsRetained:
    return cast<NSReturnsRetainedAttr>(this)->clone(C);
  case attr::Naked:
    return cast<NakedAttr>(this)->clone(C);
  case attr::NoAlias:
    return cast<NoAliasAttr>(this)->clone(C);
  case attr::NoBuiltin:
    return cast<NoBuiltinAttr>(this)->clone(C);
  case attr::NoCommon:
    return cast<NoCommonAttr>(this)->clone(C);
  case attr::NoDebug:
    return cast<NoDebugAttr>(this)->clone(C);
  case attr::NoDeref:
    return cast<NoDerefAttr>(this)->clone(C);
  case attr::NoDestroy:
    return cast<NoDestroyAttr>(this)->clone(C);
  case attr::NoDuplicate:
    return cast<NoDuplicateAttr>(this)->clone(C);
  case attr::NoEscape:
    return cast<NoEscapeAttr>(this)->clone(C);
  case attr::NoInline:
    return cast<NoInlineAttr>(this)->clone(C);
  case attr::NoInstrumentFunction:
    return cast<NoInstrumentFunctionAttr>(this)->clone(C);
  case attr::NoMicroMips:
    return cast<NoMicroMipsAttr>(this)->clone(C);
  case attr::NoMips16:
    return cast<NoMips16Attr>(this)->clone(C);
  case attr::NoReturn:
    return cast<NoReturnAttr>(this)->clone(C);
  case attr::NoSanitize:
    return cast<NoSanitizeAttr>(this)->clone(C);
  case attr::NoSpeculativeLoadHardening:
    return cast<NoSpeculativeLoadHardeningAttr>(this)->clone(C);
  case attr::NoSplitStack:
    return cast<NoSplitStackAttr>(this)->clone(C);
  case attr::NoStackProtector:
    return cast<NoStackProtectorAttr>(this)->clone(C);
  case attr::NoThreadSafetyAnalysis:
    return cast<NoThreadSafetyAnalysisAttr>(this)->clone(C);
  case attr::NoThrow:
    return cast<NoThrowAttr>(this)->clone(C);
  case attr::NoUniqueAddress:
    return cast<NoUniqueAddressAttr>(this)->clone(C);
  case attr::NonNull:
    return cast<NonNullAttr>(this)->clone(C);
  case attr::NotTailCalled:
    return cast<NotTailCalledAttr>(this)->clone(C);
  case attr::OMPAllocateDecl:
    return cast<OMPAllocateDeclAttr>(this)->clone(C);
  case attr::OMPCaptureKind:
    return cast<OMPCaptureKindAttr>(this)->clone(C);
  case attr::OMPCaptureNoInit:
    return cast<OMPCaptureNoInitAttr>(this)->clone(C);
  case attr::OMPDeclareSimdDecl:
    return cast<OMPDeclareSimdDeclAttr>(this)->clone(C);
  case attr::OMPDeclareTargetDecl:
    return cast<OMPDeclareTargetDeclAttr>(this)->clone(C);
  case attr::OMPDeclareVariant:
    return cast<OMPDeclareVariantAttr>(this)->clone(C);
  case attr::OMPReferencedVar:
    return cast<OMPReferencedVarAttr>(this)->clone(C);
  case attr::OMPThreadPrivateDecl:
    return cast<OMPThreadPrivateDeclAttr>(this)->clone(C);
  case attr::OSConsumed:
    return cast<OSConsumedAttr>(this)->clone(C);
  case attr::OSConsumesThis:
    return cast<OSConsumesThisAttr>(this)->clone(C);
  case attr::OSReturnsNotRetained:
    return cast<OSReturnsNotRetainedAttr>(this)->clone(C);
  case attr::OSReturnsRetained:
    return cast<OSReturnsRetainedAttr>(this)->clone(C);
  case attr::OSReturnsRetainedOnNonZero:
    return cast<OSReturnsRetainedOnNonZeroAttr>(this)->clone(C);
  case attr::OSReturnsRetainedOnZero:
    return cast<OSReturnsRetainedOnZeroAttr>(this)->clone(C);
  case attr::ObjCBoxable:
    return cast<ObjCBoxableAttr>(this)->clone(C);
  case attr::ObjCBridge:
    return cast<ObjCBridgeAttr>(this)->clone(C);
  case attr::ObjCBridgeMutable:
    return cast<ObjCBridgeMutableAttr>(this)->clone(C);
  case attr::ObjCBridgeRelated:
    return cast<ObjCBridgeRelatedAttr>(this)->clone(C);
  case attr::ObjCClassStub:
    return cast<ObjCClassStubAttr>(this)->clone(C);
  case attr::ObjCDesignatedInitializer:
    return cast<ObjCDesignatedInitializerAttr>(this)->clone(C);
  case attr::ObjCDirect:
    return cast<ObjCDirectAttr>(this)->clone(C);
  case attr::ObjCDirectMembers:
    return cast<ObjCDirectMembersAttr>(this)->clone(C);
  case attr::ObjCException:
    return cast<ObjCExceptionAttr>(this)->clone(C);
  case attr::ObjCExplicitProtocolImpl:
    return cast<ObjCExplicitProtocolImplAttr>(this)->clone(C);
  case attr::ObjCExternallyRetained:
    return cast<ObjCExternallyRetainedAttr>(this)->clone(C);
  case attr::ObjCGC:
    return cast<ObjCGCAttr>(this)->clone(C);
  case attr::ObjCIndependentClass:
    return cast<ObjCIndependentClassAttr>(this)->clone(C);
  case attr::ObjCInertUnsafeUnretained:
    return cast<ObjCInertUnsafeUnretainedAttr>(this)->clone(C);
  case attr::ObjCKindOf:
    return cast<ObjCKindOfAttr>(this)->clone(C);
  case attr::ObjCMethodFamily:
    return cast<ObjCMethodFamilyAttr>(this)->clone(C);
  case attr::ObjCNSObject:
    return cast<ObjCNSObjectAttr>(this)->clone(C);
  case attr::ObjCNonLazyClass:
    return cast<ObjCNonLazyClassAttr>(this)->clone(C);
  case attr::ObjCOwnership:
    return cast<ObjCOwnershipAttr>(this)->clone(C);
  case attr::ObjCPreciseLifetime:
    return cast<ObjCPreciseLifetimeAttr>(this)->clone(C);
  case attr::ObjCRequiresPropertyDefs:
    return cast<ObjCRequiresPropertyDefsAttr>(this)->clone(C);
  case attr::ObjCRequiresSuper:
    return cast<ObjCRequiresSuperAttr>(this)->clone(C);
  case attr::ObjCReturnsInnerPointer:
    return cast<ObjCReturnsInnerPointerAttr>(this)->clone(C);
  case attr::ObjCRootClass:
    return cast<ObjCRootClassAttr>(this)->clone(C);
  case attr::ObjCRuntimeName:
    return cast<ObjCRuntimeNameAttr>(this)->clone(C);
  case attr::ObjCRuntimeVisible:
    return cast<ObjCRuntimeVisibleAttr>(this)->clone(C);
  case attr::ObjCSubclassingRestricted:
    return cast<ObjCSubclassingRestrictedAttr>(this)->clone(C);
  case attr::OpenCLAccess:
    return cast<OpenCLAccessAttr>(this)->clone(C);
  case attr::OpenCLConstantAddressSpace:
    return cast<OpenCLConstantAddressSpaceAttr>(this)->clone(C);
  case attr::OpenCLGenericAddressSpace:
    return cast<OpenCLGenericAddressSpaceAttr>(this)->clone(C);
  case attr::OpenCLGlobalAddressSpace:
    return cast<OpenCLGlobalAddressSpaceAttr>(this)->clone(C);
  case attr::OpenCLIntelReqdSubGroupSize:
    return cast<OpenCLIntelReqdSubGroupSizeAttr>(this)->clone(C);
  case attr::OpenCLKernel:
    return cast<OpenCLKernelAttr>(this)->clone(C);
  case attr::OpenCLLocalAddressSpace:
    return cast<OpenCLLocalAddressSpaceAttr>(this)->clone(C);
  case attr::OpenCLPrivateAddressSpace:
    return cast<OpenCLPrivateAddressSpaceAttr>(this)->clone(C);
  case attr::OpenCLUnrollHint:
    return cast<OpenCLUnrollHintAttr>(this)->clone(C);
  case attr::OptimizeNone:
    return cast<OptimizeNoneAttr>(this)->clone(C);
  case attr::Overloadable:
    return cast<OverloadableAttr>(this)->clone(C);
  case attr::Override:
    return cast<OverrideAttr>(this)->clone(C);
  case attr::Owner:
    return cast<OwnerAttr>(this)->clone(C);
  case attr::Ownership:
    return cast<OwnershipAttr>(this)->clone(C);
  case attr::Packed:
    return cast<PackedAttr>(this)->clone(C);
  case attr::ParamTypestate:
    return cast<ParamTypestateAttr>(this)->clone(C);
  case attr::Pascal:
    return cast<PascalAttr>(this)->clone(C);
  case attr::PassObjectSize:
    return cast<PassObjectSizeAttr>(this)->clone(C);
  case attr::PatchableFunctionEntry:
    return cast<PatchableFunctionEntryAttr>(this)->clone(C);
  case attr::Pcs:
    return cast<PcsAttr>(this)->clone(C);
  case attr::Pointer:
    return cast<PointerAttr>(this)->clone(C);
  case attr::PragmaClangBSSSection:
    return cast<PragmaClangBSSSectionAttr>(this)->clone(C);
  case attr::PragmaClangDataSection:
    return cast<PragmaClangDataSectionAttr>(this)->clone(C);
  case attr::PragmaClangRelroSection:
    return cast<PragmaClangRelroSectionAttr>(this)->clone(C);
  case attr::PragmaClangRodataSection:
    return cast<PragmaClangRodataSectionAttr>(this)->clone(C);
  case attr::PragmaClangTextSection:
    return cast<PragmaClangTextSectionAttr>(this)->clone(C);
  case attr::PreserveAll:
    return cast<PreserveAllAttr>(this)->clone(C);
  case attr::PreserveMost:
    return cast<PreserveMostAttr>(this)->clone(C);
  case attr::PtGuardedBy:
    return cast<PtGuardedByAttr>(this)->clone(C);
  case attr::PtGuardedVar:
    return cast<PtGuardedVarAttr>(this)->clone(C);
  case attr::Ptr32:
    return cast<Ptr32Attr>(this)->clone(C);
  case attr::Ptr64:
    return cast<Ptr64Attr>(this)->clone(C);
  case attr::Pure:
    return cast<PureAttr>(this)->clone(C);
  case attr::RISCVInterrupt:
    return cast<RISCVInterruptAttr>(this)->clone(C);
  case attr::RegCall:
    return cast<RegCallAttr>(this)->clone(C);
  case attr::Reinitializes:
    return cast<ReinitializesAttr>(this)->clone(C);
  case attr::ReleaseCapability:
    return cast<ReleaseCapabilityAttr>(this)->clone(C);
  case attr::ReleaseHandle:
    return cast<ReleaseHandleAttr>(this)->clone(C);
  case attr::RenderScriptKernel:
    return cast<RenderScriptKernelAttr>(this)->clone(C);
  case attr::ReqdWorkGroupSize:
    return cast<ReqdWorkGroupSizeAttr>(this)->clone(C);
  case attr::RequiresCapability:
    return cast<RequiresCapabilityAttr>(this)->clone(C);
  case attr::Restrict:
    return cast<RestrictAttr>(this)->clone(C);
  case attr::ReturnTypestate:
    return cast<ReturnTypestateAttr>(this)->clone(C);
  case attr::ReturnsNonNull:
    return cast<ReturnsNonNullAttr>(this)->clone(C);
  case attr::ReturnsTwice:
    return cast<ReturnsTwiceAttr>(this)->clone(C);
  case attr::SPtr:
    return cast<SPtrAttr>(this)->clone(C);
  case attr::SYCLKernel:
    return cast<SYCLKernelAttr>(this)->clone(C);
  case attr::ScopedLockable:
    return cast<ScopedLockableAttr>(this)->clone(C);
  case attr::Section:
    return cast<SectionAttr>(this)->clone(C);
  case attr::SelectAny:
    return cast<SelectAnyAttr>(this)->clone(C);
  case attr::Sentinel:
    return cast<SentinelAttr>(this)->clone(C);
  case attr::SetTypestate:
    return cast<SetTypestateAttr>(this)->clone(C);
  case attr::SharedTrylockFunction:
    return cast<SharedTrylockFunctionAttr>(this)->clone(C);
  case attr::SpeculativeLoadHardening:
    return cast<SpeculativeLoadHardeningAttr>(this)->clone(C);
  case attr::StdCall:
    return cast<StdCallAttr>(this)->clone(C);
  case attr::Suppress:
    return cast<SuppressAttr>(this)->clone(C);
  case attr::SwiftCall:
    return cast<SwiftCallAttr>(this)->clone(C);
  case attr::SwiftContext:
    return cast<SwiftContextAttr>(this)->clone(C);
  case attr::SwiftErrorResult:
    return cast<SwiftErrorResultAttr>(this)->clone(C);
  case attr::SwiftIndirectResult:
    return cast<SwiftIndirectResultAttr>(this)->clone(C);
  case attr::SysVABI:
    return cast<SysVABIAttr>(this)->clone(C);
  case attr::TLSModel:
    return cast<TLSModelAttr>(this)->clone(C);
  case attr::Target:
    return cast<TargetAttr>(this)->clone(C);
  case attr::TestTypestate:
    return cast<TestTypestateAttr>(this)->clone(C);
  case attr::ThisCall:
    return cast<ThisCallAttr>(this)->clone(C);
  case attr::Thread:
    return cast<ThreadAttr>(this)->clone(C);
  case attr::TransparentUnion:
    return cast<TransparentUnionAttr>(this)->clone(C);
  case attr::TrivialABI:
    return cast<TrivialABIAttr>(this)->clone(C);
  case attr::TryAcquireCapability:
    return cast<TryAcquireCapabilityAttr>(this)->clone(C);
  case attr::TypeNonNull:
    return cast<TypeNonNullAttr>(this)->clone(C);
  case attr::TypeNullUnspecified:
    return cast<TypeNullUnspecifiedAttr>(this)->clone(C);
  case attr::TypeNullable:
    return cast<TypeNullableAttr>(this)->clone(C);
  case attr::TypeTagForDatatype:
    return cast<TypeTagForDatatypeAttr>(this)->clone(C);
  case attr::TypeVisibility:
    return cast<TypeVisibilityAttr>(this)->clone(C);
  case attr::UPtr:
    return cast<UPtrAttr>(this)->clone(C);
  case attr::Unavailable:
    return cast<UnavailableAttr>(this)->clone(C);
  case attr::Uninitialized:
    return cast<UninitializedAttr>(this)->clone(C);
  case attr::Unused:
    return cast<UnusedAttr>(this)->clone(C);
  case attr::UseHandle:
    return cast<UseHandleAttr>(this)->clone(C);
  case attr::Used:
    return cast<UsedAttr>(this)->clone(C);
  case attr::Uuid:
    return cast<UuidAttr>(this)->clone(C);
  case attr::VecReturn:
    return cast<VecReturnAttr>(this)->clone(C);
  case attr::VecTypeHint:
    return cast<VecTypeHintAttr>(this)->clone(C);
  case attr::VectorCall:
    return cast<VectorCallAttr>(this)->clone(C);
  case attr::Visibility:
    return cast<VisibilityAttr>(this)->clone(C);
  case attr::WarnUnused:
    return cast<WarnUnusedAttr>(this)->clone(C);
  case attr::WarnUnusedResult:
    return cast<WarnUnusedResultAttr>(this)->clone(C);
  case attr::Weak:
    return cast<WeakAttr>(this)->clone(C);
  case attr::WeakImport:
    return cast<WeakImportAttr>(this)->clone(C);
  case attr::WeakRef:
    return cast<WeakRefAttr>(this)->clone(C);
  case attr::WebAssemblyExportName:
    return cast<WebAssemblyExportNameAttr>(this)->clone(C);
  case attr::WebAssemblyImportModule:
    return cast<WebAssemblyImportModuleAttr>(this)->clone(C);
  case attr::WebAssemblyImportName:
    return cast<WebAssemblyImportNameAttr>(this)->clone(C);
  case attr::WorkGroupSizeHint:
    return cast<WorkGroupSizeHintAttr>(this)->clone(C);
  case attr::X86ForceAlignArgPointer:
    return cast<X86ForceAlignArgPointerAttr>(this)->clone(C);
  case attr::XRayInstrument:
    return cast<XRayInstrumentAttr>(this)->clone(C);
  case attr::XRayLogArgs:
    return cast<XRayLogArgsAttr>(this)->clone(C);
  }
  llvm_unreachable("Unexpected attribute kind!");
}

void Attr::printPretty(raw_ostream &OS, const PrintingPolicy &Policy) const {
  switch (getKind()) {
  case attr::AArch64VectorPcs:
    return cast<AArch64VectorPcsAttr>(this)->printPretty(OS, Policy);
  case attr::AMDGPUFlatWorkGroupSize:
    return cast<AMDGPUFlatWorkGroupSizeAttr>(this)->printPretty(OS, Policy);
  case attr::AMDGPUNumSGPR:
    return cast<AMDGPUNumSGPRAttr>(this)->printPretty(OS, Policy);
  case attr::AMDGPUNumVGPR:
    return cast<AMDGPUNumVGPRAttr>(this)->printPretty(OS, Policy);
  case attr::AMDGPUWavesPerEU:
    return cast<AMDGPUWavesPerEUAttr>(this)->printPretty(OS, Policy);
  case attr::ARMInterrupt:
    return cast<ARMInterruptAttr>(this)->printPretty(OS, Policy);
  case attr::AVRInterrupt:
    return cast<AVRInterruptAttr>(this)->printPretty(OS, Policy);
  case attr::AVRSignal:
    return cast<AVRSignalAttr>(this)->printPretty(OS, Policy);
  case attr::AbiTag:
    return cast<AbiTagAttr>(this)->printPretty(OS, Policy);
  case attr::AcquireCapability:
    return cast<AcquireCapabilityAttr>(this)->printPretty(OS, Policy);
  case attr::AcquireHandle:
    return cast<AcquireHandleAttr>(this)->printPretty(OS, Policy);
  case attr::AcquiredAfter:
    return cast<AcquiredAfterAttr>(this)->printPretty(OS, Policy);
  case attr::AcquiredBefore:
    return cast<AcquiredBeforeAttr>(this)->printPretty(OS, Policy);
  case attr::AddressSpace:
    return cast<AddressSpaceAttr>(this)->printPretty(OS, Policy);
  case attr::Alias:
    return cast<AliasAttr>(this)->printPretty(OS, Policy);
  case attr::AlignMac68k:
    return cast<AlignMac68kAttr>(this)->printPretty(OS, Policy);
  case attr::AlignValue:
    return cast<AlignValueAttr>(this)->printPretty(OS, Policy);
  case attr::Aligned:
    return cast<AlignedAttr>(this)->printPretty(OS, Policy);
  case attr::AllocAlign:
    return cast<AllocAlignAttr>(this)->printPretty(OS, Policy);
  case attr::AllocSize:
    return cast<AllocSizeAttr>(this)->printPretty(OS, Policy);
  case attr::AlwaysDestroy:
    return cast<AlwaysDestroyAttr>(this)->printPretty(OS, Policy);
  case attr::AlwaysInline:
    return cast<AlwaysInlineAttr>(this)->printPretty(OS, Policy);
  case attr::AnalyzerNoReturn:
    return cast<AnalyzerNoReturnAttr>(this)->printPretty(OS, Policy);
  case attr::Annotate:
    return cast<AnnotateAttr>(this)->printPretty(OS, Policy);
  case attr::AnyX86Interrupt:
    return cast<AnyX86InterruptAttr>(this)->printPretty(OS, Policy);
  case attr::AnyX86NoCallerSavedRegisters:
    return cast<AnyX86NoCallerSavedRegistersAttr>(this)->printPretty(OS, Policy);
  case attr::AnyX86NoCfCheck:
    return cast<AnyX86NoCfCheckAttr>(this)->printPretty(OS, Policy);
  case attr::ArcWeakrefUnavailable:
    return cast<ArcWeakrefUnavailableAttr>(this)->printPretty(OS, Policy);
  case attr::ArgumentWithTypeTag:
    return cast<ArgumentWithTypeTagAttr>(this)->printPretty(OS, Policy);
  case attr::ArmMveAlias:
    return cast<ArmMveAliasAttr>(this)->printPretty(OS, Policy);
  case attr::ArmMveStrictPolymorphism:
    return cast<ArmMveStrictPolymorphismAttr>(this)->printPretty(OS, Policy);
  case attr::Artificial:
    return cast<ArtificialAttr>(this)->printPretty(OS, Policy);
  case attr::AsmLabel:
    return cast<AsmLabelAttr>(this)->printPretty(OS, Policy);
  case attr::AssertCapability:
    return cast<AssertCapabilityAttr>(this)->printPretty(OS, Policy);
  case attr::AssertExclusiveLock:
    return cast<AssertExclusiveLockAttr>(this)->printPretty(OS, Policy);
  case attr::AssertSharedLock:
    return cast<AssertSharedLockAttr>(this)->printPretty(OS, Policy);
  case attr::AssumeAligned:
    return cast<AssumeAlignedAttr>(this)->printPretty(OS, Policy);
  case attr::Availability:
    return cast<AvailabilityAttr>(this)->printPretty(OS, Policy);
  case attr::BPFPreserveAccessIndex:
    return cast<BPFPreserveAccessIndexAttr>(this)->printPretty(OS, Policy);
  case attr::Blocks:
    return cast<BlocksAttr>(this)->printPretty(OS, Policy);
  case attr::C11NoReturn:
    return cast<C11NoReturnAttr>(this)->printPretty(OS, Policy);
  case attr::CDecl:
    return cast<CDeclAttr>(this)->printPretty(OS, Policy);
  case attr::CFAuditedTransfer:
    return cast<CFAuditedTransferAttr>(this)->printPretty(OS, Policy);
  case attr::CFConsumed:
    return cast<CFConsumedAttr>(this)->printPretty(OS, Policy);
  case attr::CFGuard:
    return cast<CFGuardAttr>(this)->printPretty(OS, Policy);
  case attr::CFICanonicalJumpTable:
    return cast<CFICanonicalJumpTableAttr>(this)->printPretty(OS, Policy);
  case attr::CFReturnsNotRetained:
    return cast<CFReturnsNotRetainedAttr>(this)->printPretty(OS, Policy);
  case attr::CFReturnsRetained:
    return cast<CFReturnsRetainedAttr>(this)->printPretty(OS, Policy);
  case attr::CFUnknownTransfer:
    return cast<CFUnknownTransferAttr>(this)->printPretty(OS, Policy);
  case attr::CPUDispatch:
    return cast<CPUDispatchAttr>(this)->printPretty(OS, Policy);
  case attr::CPUSpecific:
    return cast<CPUSpecificAttr>(this)->printPretty(OS, Policy);
  case attr::CUDAConstant:
    return cast<CUDAConstantAttr>(this)->printPretty(OS, Policy);
  case attr::CUDADevice:
    return cast<CUDADeviceAttr>(this)->printPretty(OS, Policy);
  case attr::CUDAGlobal:
    return cast<CUDAGlobalAttr>(this)->printPretty(OS, Policy);
  case attr::CUDAHost:
    return cast<CUDAHostAttr>(this)->printPretty(OS, Policy);
  case attr::CUDAInvalidTarget:
    return cast<CUDAInvalidTargetAttr>(this)->printPretty(OS, Policy);
  case attr::CUDALaunchBounds:
    return cast<CUDALaunchBoundsAttr>(this)->printPretty(OS, Policy);
  case attr::CUDAShared:
    return cast<CUDASharedAttr>(this)->printPretty(OS, Policy);
  case attr::CXX11NoReturn:
    return cast<CXX11NoReturnAttr>(this)->printPretty(OS, Policy);
  case attr::CallableWhen:
    return cast<CallableWhenAttr>(this)->printPretty(OS, Policy);
  case attr::Callback:
    return cast<CallbackAttr>(this)->printPretty(OS, Policy);
  case attr::Capability:
    return cast<CapabilityAttr>(this)->printPretty(OS, Policy);
  case attr::CapturedRecord:
    return cast<CapturedRecordAttr>(this)->printPretty(OS, Policy);
  case attr::CarriesDependency:
    return cast<CarriesDependencyAttr>(this)->printPretty(OS, Policy);
  case attr::Cleanup:
    return cast<CleanupAttr>(this)->printPretty(OS, Policy);
  case attr::CodeSeg:
    return cast<CodeSegAttr>(this)->printPretty(OS, Policy);
  case attr::Cold:
    return cast<ColdAttr>(this)->printPretty(OS, Policy);
  case attr::Common:
    return cast<CommonAttr>(this)->printPretty(OS, Policy);
  case attr::Const:
    return cast<ConstAttr>(this)->printPretty(OS, Policy);
  case attr::ConstInit:
    return cast<ConstInitAttr>(this)->printPretty(OS, Policy);
  case attr::Constructor:
    return cast<ConstructorAttr>(this)->printPretty(OS, Policy);
  case attr::Consumable:
    return cast<ConsumableAttr>(this)->printPretty(OS, Policy);
  case attr::ConsumableAutoCast:
    return cast<ConsumableAutoCastAttr>(this)->printPretty(OS, Policy);
  case attr::ConsumableSetOnRead:
    return cast<ConsumableSetOnReadAttr>(this)->printPretty(OS, Policy);
  case attr::Convergent:
    return cast<ConvergentAttr>(this)->printPretty(OS, Policy);
  case attr::DLLExport:
    return cast<DLLExportAttr>(this)->printPretty(OS, Policy);
  case attr::DLLExportStaticLocal:
    return cast<DLLExportStaticLocalAttr>(this)->printPretty(OS, Policy);
  case attr::DLLImport:
    return cast<DLLImportAttr>(this)->printPretty(OS, Policy);
  case attr::DLLImportStaticLocal:
    return cast<DLLImportStaticLocalAttr>(this)->printPretty(OS, Policy);
  case attr::Deprecated:
    return cast<DeprecatedAttr>(this)->printPretty(OS, Policy);
  case attr::Destructor:
    return cast<DestructorAttr>(this)->printPretty(OS, Policy);
  case attr::DiagnoseIf:
    return cast<DiagnoseIfAttr>(this)->printPretty(OS, Policy);
  case attr::DisableTailCalls:
    return cast<DisableTailCallsAttr>(this)->printPretty(OS, Policy);
  case attr::EmptyBases:
    return cast<EmptyBasesAttr>(this)->printPretty(OS, Policy);
  case attr::EnableIf:
    return cast<EnableIfAttr>(this)->printPretty(OS, Policy);
  case attr::EnumExtensibility:
    return cast<EnumExtensibilityAttr>(this)->printPretty(OS, Policy);
  case attr::ExcludeFromExplicitInstantiation:
    return cast<ExcludeFromExplicitInstantiationAttr>(this)->printPretty(OS, Policy);
  case attr::ExclusiveTrylockFunction:
    return cast<ExclusiveTrylockFunctionAttr>(this)->printPretty(OS, Policy);
  case attr::ExternalSourceSymbol:
    return cast<ExternalSourceSymbolAttr>(this)->printPretty(OS, Policy);
  case attr::FallThrough:
    return cast<FallThroughAttr>(this)->printPretty(OS, Policy);
  case attr::FastCall:
    return cast<FastCallAttr>(this)->printPretty(OS, Policy);
  case attr::Final:
    return cast<FinalAttr>(this)->printPretty(OS, Policy);
  case attr::FlagEnum:
    return cast<FlagEnumAttr>(this)->printPretty(OS, Policy);
  case attr::Flatten:
    return cast<FlattenAttr>(this)->printPretty(OS, Policy);
  case attr::Format:
    return cast<FormatAttr>(this)->printPretty(OS, Policy);
  case attr::FormatArg:
    return cast<FormatArgAttr>(this)->printPretty(OS, Policy);
  case attr::GNUInline:
    return cast<GNUInlineAttr>(this)->printPretty(OS, Policy);
  case attr::GuardedBy:
    return cast<GuardedByAttr>(this)->printPretty(OS, Policy);
  case attr::GuardedVar:
    return cast<GuardedVarAttr>(this)->printPretty(OS, Policy);
  case attr::HIPPinnedShadow:
    return cast<HIPPinnedShadowAttr>(this)->printPretty(OS, Policy);
  case attr::Hot:
    return cast<HotAttr>(this)->printPretty(OS, Policy);
  case attr::IBAction:
    return cast<IBActionAttr>(this)->printPretty(OS, Policy);
  case attr::IBOutlet:
    return cast<IBOutletAttr>(this)->printPretty(OS, Policy);
  case attr::IBOutletCollection:
    return cast<IBOutletCollectionAttr>(this)->printPretty(OS, Policy);
  case attr::IFunc:
    return cast<IFuncAttr>(this)->printPretty(OS, Policy);
  case attr::InitPriority:
    return cast<InitPriorityAttr>(this)->printPretty(OS, Policy);
  case attr::InitSeg:
    return cast<InitSegAttr>(this)->printPretty(OS, Policy);
  case attr::IntelOclBicc:
    return cast<IntelOclBiccAttr>(this)->printPretty(OS, Policy);
  case attr::InternalLinkage:
    return cast<InternalLinkageAttr>(this)->printPretty(OS, Policy);
  case attr::LTOVisibilityPublic:
    return cast<LTOVisibilityPublicAttr>(this)->printPretty(OS, Policy);
  case attr::LayoutVersion:
    return cast<LayoutVersionAttr>(this)->printPretty(OS, Policy);
  case attr::LifetimeBound:
    return cast<LifetimeBoundAttr>(this)->printPretty(OS, Policy);
  case attr::LockReturned:
    return cast<LockReturnedAttr>(this)->printPretty(OS, Policy);
  case attr::LocksExcluded:
    return cast<LocksExcludedAttr>(this)->printPretty(OS, Policy);
  case attr::LoopHint:
    return cast<LoopHintAttr>(this)->printPretty(OS, Policy);
  case attr::MIGServerRoutine:
    return cast<MIGServerRoutineAttr>(this)->printPretty(OS, Policy);
  case attr::MSABI:
    return cast<MSABIAttr>(this)->printPretty(OS, Policy);
  case attr::MSAllocator:
    return cast<MSAllocatorAttr>(this)->printPretty(OS, Policy);
  case attr::MSInheritance:
    return cast<MSInheritanceAttr>(this)->printPretty(OS, Policy);
  case attr::MSNoVTable:
    return cast<MSNoVTableAttr>(this)->printPretty(OS, Policy);
  case attr::MSP430Interrupt:
    return cast<MSP430InterruptAttr>(this)->printPretty(OS, Policy);
  case attr::MSStruct:
    return cast<MSStructAttr>(this)->printPretty(OS, Policy);
  case attr::MSVtorDisp:
    return cast<MSVtorDispAttr>(this)->printPretty(OS, Policy);
  case attr::MaxFieldAlignment:
    return cast<MaxFieldAlignmentAttr>(this)->printPretty(OS, Policy);
  case attr::MayAlias:
    return cast<MayAliasAttr>(this)->printPretty(OS, Policy);
  case attr::MicroMips:
    return cast<MicroMipsAttr>(this)->printPretty(OS, Policy);
  case attr::MinSize:
    return cast<MinSizeAttr>(this)->printPretty(OS, Policy);
  case attr::MinVectorWidth:
    return cast<MinVectorWidthAttr>(this)->printPretty(OS, Policy);
  case attr::Mips16:
    return cast<Mips16Attr>(this)->printPretty(OS, Policy);
  case attr::MipsInterrupt:
    return cast<MipsInterruptAttr>(this)->printPretty(OS, Policy);
  case attr::MipsLongCall:
    return cast<MipsLongCallAttr>(this)->printPretty(OS, Policy);
  case attr::MipsShortCall:
    return cast<MipsShortCallAttr>(this)->printPretty(OS, Policy);
  case attr::Mode:
    return cast<ModeAttr>(this)->printPretty(OS, Policy);
  case attr::NSConsumed:
    return cast<NSConsumedAttr>(this)->printPretty(OS, Policy);
  case attr::NSConsumesSelf:
    return cast<NSConsumesSelfAttr>(this)->printPretty(OS, Policy);
  case attr::NSReturnsAutoreleased:
    return cast<NSReturnsAutoreleasedAttr>(this)->printPretty(OS, Policy);
  case attr::NSReturnsNotRetained:
    return cast<NSReturnsNotRetainedAttr>(this)->printPretty(OS, Policy);
  case attr::NSReturnsRetained:
    return cast<NSReturnsRetainedAttr>(this)->printPretty(OS, Policy);
  case attr::Naked:
    return cast<NakedAttr>(this)->printPretty(OS, Policy);
  case attr::NoAlias:
    return cast<NoAliasAttr>(this)->printPretty(OS, Policy);
  case attr::NoBuiltin:
    return cast<NoBuiltinAttr>(this)->printPretty(OS, Policy);
  case attr::NoCommon:
    return cast<NoCommonAttr>(this)->printPretty(OS, Policy);
  case attr::NoDebug:
    return cast<NoDebugAttr>(this)->printPretty(OS, Policy);
  case attr::NoDeref:
    return cast<NoDerefAttr>(this)->printPretty(OS, Policy);
  case attr::NoDestroy:
    return cast<NoDestroyAttr>(this)->printPretty(OS, Policy);
  case attr::NoDuplicate:
    return cast<NoDuplicateAttr>(this)->printPretty(OS, Policy);
  case attr::NoEscape:
    return cast<NoEscapeAttr>(this)->printPretty(OS, Policy);
  case attr::NoInline:
    return cast<NoInlineAttr>(this)->printPretty(OS, Policy);
  case attr::NoInstrumentFunction:
    return cast<NoInstrumentFunctionAttr>(this)->printPretty(OS, Policy);
  case attr::NoMicroMips:
    return cast<NoMicroMipsAttr>(this)->printPretty(OS, Policy);
  case attr::NoMips16:
    return cast<NoMips16Attr>(this)->printPretty(OS, Policy);
  case attr::NoReturn:
    return cast<NoReturnAttr>(this)->printPretty(OS, Policy);
  case attr::NoSanitize:
    return cast<NoSanitizeAttr>(this)->printPretty(OS, Policy);
  case attr::NoSpeculativeLoadHardening:
    return cast<NoSpeculativeLoadHardeningAttr>(this)->printPretty(OS, Policy);
  case attr::NoSplitStack:
    return cast<NoSplitStackAttr>(this)->printPretty(OS, Policy);
  case attr::NoStackProtector:
    return cast<NoStackProtectorAttr>(this)->printPretty(OS, Policy);
  case attr::NoThreadSafetyAnalysis:
    return cast<NoThreadSafetyAnalysisAttr>(this)->printPretty(OS, Policy);
  case attr::NoThrow:
    return cast<NoThrowAttr>(this)->printPretty(OS, Policy);
  case attr::NoUniqueAddress:
    return cast<NoUniqueAddressAttr>(this)->printPretty(OS, Policy);
  case attr::NonNull:
    return cast<NonNullAttr>(this)->printPretty(OS, Policy);
  case attr::NotTailCalled:
    return cast<NotTailCalledAttr>(this)->printPretty(OS, Policy);
  case attr::OMPAllocateDecl:
    return cast<OMPAllocateDeclAttr>(this)->printPretty(OS, Policy);
  case attr::OMPCaptureKind:
    return cast<OMPCaptureKindAttr>(this)->printPretty(OS, Policy);
  case attr::OMPCaptureNoInit:
    return cast<OMPCaptureNoInitAttr>(this)->printPretty(OS, Policy);
  case attr::OMPDeclareSimdDecl:
    return cast<OMPDeclareSimdDeclAttr>(this)->printPretty(OS, Policy);
  case attr::OMPDeclareTargetDecl:
    return cast<OMPDeclareTargetDeclAttr>(this)->printPretty(OS, Policy);
  case attr::OMPDeclareVariant:
    return cast<OMPDeclareVariantAttr>(this)->printPretty(OS, Policy);
  case attr::OMPReferencedVar:
    return cast<OMPReferencedVarAttr>(this)->printPretty(OS, Policy);
  case attr::OMPThreadPrivateDecl:
    return cast<OMPThreadPrivateDeclAttr>(this)->printPretty(OS, Policy);
  case attr::OSConsumed:
    return cast<OSConsumedAttr>(this)->printPretty(OS, Policy);
  case attr::OSConsumesThis:
    return cast<OSConsumesThisAttr>(this)->printPretty(OS, Policy);
  case attr::OSReturnsNotRetained:
    return cast<OSReturnsNotRetainedAttr>(this)->printPretty(OS, Policy);
  case attr::OSReturnsRetained:
    return cast<OSReturnsRetainedAttr>(this)->printPretty(OS, Policy);
  case attr::OSReturnsRetainedOnNonZero:
    return cast<OSReturnsRetainedOnNonZeroAttr>(this)->printPretty(OS, Policy);
  case attr::OSReturnsRetainedOnZero:
    return cast<OSReturnsRetainedOnZeroAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCBoxable:
    return cast<ObjCBoxableAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCBridge:
    return cast<ObjCBridgeAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCBridgeMutable:
    return cast<ObjCBridgeMutableAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCBridgeRelated:
    return cast<ObjCBridgeRelatedAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCClassStub:
    return cast<ObjCClassStubAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCDesignatedInitializer:
    return cast<ObjCDesignatedInitializerAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCDirect:
    return cast<ObjCDirectAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCDirectMembers:
    return cast<ObjCDirectMembersAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCException:
    return cast<ObjCExceptionAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCExplicitProtocolImpl:
    return cast<ObjCExplicitProtocolImplAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCExternallyRetained:
    return cast<ObjCExternallyRetainedAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCGC:
    return cast<ObjCGCAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCIndependentClass:
    return cast<ObjCIndependentClassAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCInertUnsafeUnretained:
    return cast<ObjCInertUnsafeUnretainedAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCKindOf:
    return cast<ObjCKindOfAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCMethodFamily:
    return cast<ObjCMethodFamilyAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCNSObject:
    return cast<ObjCNSObjectAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCNonLazyClass:
    return cast<ObjCNonLazyClassAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCOwnership:
    return cast<ObjCOwnershipAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCPreciseLifetime:
    return cast<ObjCPreciseLifetimeAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCRequiresPropertyDefs:
    return cast<ObjCRequiresPropertyDefsAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCRequiresSuper:
    return cast<ObjCRequiresSuperAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCReturnsInnerPointer:
    return cast<ObjCReturnsInnerPointerAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCRootClass:
    return cast<ObjCRootClassAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCRuntimeName:
    return cast<ObjCRuntimeNameAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCRuntimeVisible:
    return cast<ObjCRuntimeVisibleAttr>(this)->printPretty(OS, Policy);
  case attr::ObjCSubclassingRestricted:
    return cast<ObjCSubclassingRestrictedAttr>(this)->printPretty(OS, Policy);
  case attr::OpenCLAccess:
    return cast<OpenCLAccessAttr>(this)->printPretty(OS, Policy);
  case attr::OpenCLConstantAddressSpace:
    return cast<OpenCLConstantAddressSpaceAttr>(this)->printPretty(OS, Policy);
  case attr::OpenCLGenericAddressSpace:
    return cast<OpenCLGenericAddressSpaceAttr>(this)->printPretty(OS, Policy);
  case attr::OpenCLGlobalAddressSpace:
    return cast<OpenCLGlobalAddressSpaceAttr>(this)->printPretty(OS, Policy);
  case attr::OpenCLIntelReqdSubGroupSize:
    return cast<OpenCLIntelReqdSubGroupSizeAttr>(this)->printPretty(OS, Policy);
  case attr::OpenCLKernel:
    return cast<OpenCLKernelAttr>(this)->printPretty(OS, Policy);
  case attr::OpenCLLocalAddressSpace:
    return cast<OpenCLLocalAddressSpaceAttr>(this)->printPretty(OS, Policy);
  case attr::OpenCLPrivateAddressSpace:
    return cast<OpenCLPrivateAddressSpaceAttr>(this)->printPretty(OS, Policy);
  case attr::OpenCLUnrollHint:
    return cast<OpenCLUnrollHintAttr>(this)->printPretty(OS, Policy);
  case attr::OptimizeNone:
    return cast<OptimizeNoneAttr>(this)->printPretty(OS, Policy);
  case attr::Overloadable:
    return cast<OverloadableAttr>(this)->printPretty(OS, Policy);
  case attr::Override:
    return cast<OverrideAttr>(this)->printPretty(OS, Policy);
  case attr::Owner:
    return cast<OwnerAttr>(this)->printPretty(OS, Policy);
  case attr::Ownership:
    return cast<OwnershipAttr>(this)->printPretty(OS, Policy);
  case attr::Packed:
    return cast<PackedAttr>(this)->printPretty(OS, Policy);
  case attr::ParamTypestate:
    return cast<ParamTypestateAttr>(this)->printPretty(OS, Policy);
  case attr::Pascal:
    return cast<PascalAttr>(this)->printPretty(OS, Policy);
  case attr::PassObjectSize:
    return cast<PassObjectSizeAttr>(this)->printPretty(OS, Policy);
  case attr::PatchableFunctionEntry:
    return cast<PatchableFunctionEntryAttr>(this)->printPretty(OS, Policy);
  case attr::Pcs:
    return cast<PcsAttr>(this)->printPretty(OS, Policy);
  case attr::Pointer:
    return cast<PointerAttr>(this)->printPretty(OS, Policy);
  case attr::PragmaClangBSSSection:
    return cast<PragmaClangBSSSectionAttr>(this)->printPretty(OS, Policy);
  case attr::PragmaClangDataSection:
    return cast<PragmaClangDataSectionAttr>(this)->printPretty(OS, Policy);
  case attr::PragmaClangRelroSection:
    return cast<PragmaClangRelroSectionAttr>(this)->printPretty(OS, Policy);
  case attr::PragmaClangRodataSection:
    return cast<PragmaClangRodataSectionAttr>(this)->printPretty(OS, Policy);
  case attr::PragmaClangTextSection:
    return cast<PragmaClangTextSectionAttr>(this)->printPretty(OS, Policy);
  case attr::PreserveAll:
    return cast<PreserveAllAttr>(this)->printPretty(OS, Policy);
  case attr::PreserveMost:
    return cast<PreserveMostAttr>(this)->printPretty(OS, Policy);
  case attr::PtGuardedBy:
    return cast<PtGuardedByAttr>(this)->printPretty(OS, Policy);
  case attr::PtGuardedVar:
    return cast<PtGuardedVarAttr>(this)->printPretty(OS, Policy);
  case attr::Ptr32:
    return cast<Ptr32Attr>(this)->printPretty(OS, Policy);
  case attr::Ptr64:
    return cast<Ptr64Attr>(this)->printPretty(OS, Policy);
  case attr::Pure:
    return cast<PureAttr>(this)->printPretty(OS, Policy);
  case attr::RISCVInterrupt:
    return cast<RISCVInterruptAttr>(this)->printPretty(OS, Policy);
  case attr::RegCall:
    return cast<RegCallAttr>(this)->printPretty(OS, Policy);
  case attr::Reinitializes:
    return cast<ReinitializesAttr>(this)->printPretty(OS, Policy);
  case attr::ReleaseCapability:
    return cast<ReleaseCapabilityAttr>(this)->printPretty(OS, Policy);
  case attr::ReleaseHandle:
    return cast<ReleaseHandleAttr>(this)->printPretty(OS, Policy);
  case attr::RenderScriptKernel:
    return cast<RenderScriptKernelAttr>(this)->printPretty(OS, Policy);
  case attr::ReqdWorkGroupSize:
    return cast<ReqdWorkGroupSizeAttr>(this)->printPretty(OS, Policy);
  case attr::RequiresCapability:
    return cast<RequiresCapabilityAttr>(this)->printPretty(OS, Policy);
  case attr::Restrict:
    return cast<RestrictAttr>(this)->printPretty(OS, Policy);
  case attr::ReturnTypestate:
    return cast<ReturnTypestateAttr>(this)->printPretty(OS, Policy);
  case attr::ReturnsNonNull:
    return cast<ReturnsNonNullAttr>(this)->printPretty(OS, Policy);
  case attr::ReturnsTwice:
    return cast<ReturnsTwiceAttr>(this)->printPretty(OS, Policy);
  case attr::SPtr:
    return cast<SPtrAttr>(this)->printPretty(OS, Policy);
  case attr::SYCLKernel:
    return cast<SYCLKernelAttr>(this)->printPretty(OS, Policy);
  case attr::ScopedLockable:
    return cast<ScopedLockableAttr>(this)->printPretty(OS, Policy);
  case attr::Section:
    return cast<SectionAttr>(this)->printPretty(OS, Policy);
  case attr::SelectAny:
    return cast<SelectAnyAttr>(this)->printPretty(OS, Policy);
  case attr::Sentinel:
    return cast<SentinelAttr>(this)->printPretty(OS, Policy);
  case attr::SetTypestate:
    return cast<SetTypestateAttr>(this)->printPretty(OS, Policy);
  case attr::SharedTrylockFunction:
    return cast<SharedTrylockFunctionAttr>(this)->printPretty(OS, Policy);
  case attr::SpeculativeLoadHardening:
    return cast<SpeculativeLoadHardeningAttr>(this)->printPretty(OS, Policy);
  case attr::StdCall:
    return cast<StdCallAttr>(this)->printPretty(OS, Policy);
  case attr::Suppress:
    return cast<SuppressAttr>(this)->printPretty(OS, Policy);
  case attr::SwiftCall:
    return cast<SwiftCallAttr>(this)->printPretty(OS, Policy);
  case attr::SwiftContext:
    return cast<SwiftContextAttr>(this)->printPretty(OS, Policy);
  case attr::SwiftErrorResult:
    return cast<SwiftErrorResultAttr>(this)->printPretty(OS, Policy);
  case attr::SwiftIndirectResult:
    return cast<SwiftIndirectResultAttr>(this)->printPretty(OS, Policy);
  case attr::SysVABI:
    return cast<SysVABIAttr>(this)->printPretty(OS, Policy);
  case attr::TLSModel:
    return cast<TLSModelAttr>(this)->printPretty(OS, Policy);
  case attr::Target:
    return cast<TargetAttr>(this)->printPretty(OS, Policy);
  case attr::TestTypestate:
    return cast<TestTypestateAttr>(this)->printPretty(OS, Policy);
  case attr::ThisCall:
    return cast<ThisCallAttr>(this)->printPretty(OS, Policy);
  case attr::Thread:
    return cast<ThreadAttr>(this)->printPretty(OS, Policy);
  case attr::TransparentUnion:
    return cast<TransparentUnionAttr>(this)->printPretty(OS, Policy);
  case attr::TrivialABI:
    return cast<TrivialABIAttr>(this)->printPretty(OS, Policy);
  case attr::TryAcquireCapability:
    return cast<TryAcquireCapabilityAttr>(this)->printPretty(OS, Policy);
  case attr::TypeNonNull:
    return cast<TypeNonNullAttr>(this)->printPretty(OS, Policy);
  case attr::TypeNullUnspecified:
    return cast<TypeNullUnspecifiedAttr>(this)->printPretty(OS, Policy);
  case attr::TypeNullable:
    return cast<TypeNullableAttr>(this)->printPretty(OS, Policy);
  case attr::TypeTagForDatatype:
    return cast<TypeTagForDatatypeAttr>(this)->printPretty(OS, Policy);
  case attr::TypeVisibility:
    return cast<TypeVisibilityAttr>(this)->printPretty(OS, Policy);
  case attr::UPtr:
    return cast<UPtrAttr>(this)->printPretty(OS, Policy);
  case attr::Unavailable:
    return cast<UnavailableAttr>(this)->printPretty(OS, Policy);
  case attr::Uninitialized:
    return cast<UninitializedAttr>(this)->printPretty(OS, Policy);
  case attr::Unused:
    return cast<UnusedAttr>(this)->printPretty(OS, Policy);
  case attr::UseHandle:
    return cast<UseHandleAttr>(this)->printPretty(OS, Policy);
  case attr::Used:
    return cast<UsedAttr>(this)->printPretty(OS, Policy);
  case attr::Uuid:
    return cast<UuidAttr>(this)->printPretty(OS, Policy);
  case attr::VecReturn:
    return cast<VecReturnAttr>(this)->printPretty(OS, Policy);
  case attr::VecTypeHint:
    return cast<VecTypeHintAttr>(this)->printPretty(OS, Policy);
  case attr::VectorCall:
    return cast<VectorCallAttr>(this)->printPretty(OS, Policy);
  case attr::Visibility:
    return cast<VisibilityAttr>(this)->printPretty(OS, Policy);
  case attr::WarnUnused:
    return cast<WarnUnusedAttr>(this)->printPretty(OS, Policy);
  case attr::WarnUnusedResult:
    return cast<WarnUnusedResultAttr>(this)->printPretty(OS, Policy);
  case attr::Weak:
    return cast<WeakAttr>(this)->printPretty(OS, Policy);
  case attr::WeakImport:
    return cast<WeakImportAttr>(this)->printPretty(OS, Policy);
  case attr::WeakRef:
    return cast<WeakRefAttr>(this)->printPretty(OS, Policy);
  case attr::WebAssemblyExportName:
    return cast<WebAssemblyExportNameAttr>(this)->printPretty(OS, Policy);
  case attr::WebAssemblyImportModule:
    return cast<WebAssemblyImportModuleAttr>(this)->printPretty(OS, Policy);
  case attr::WebAssemblyImportName:
    return cast<WebAssemblyImportNameAttr>(this)->printPretty(OS, Policy);
  case attr::WorkGroupSizeHint:
    return cast<WorkGroupSizeHintAttr>(this)->printPretty(OS, Policy);
  case attr::X86ForceAlignArgPointer:
    return cast<X86ForceAlignArgPointerAttr>(this)->printPretty(OS, Policy);
  case attr::XRayInstrument:
    return cast<XRayInstrumentAttr>(this)->printPretty(OS, Policy);
  case attr::XRayLogArgs:
    return cast<XRayLogArgsAttr>(this)->printPretty(OS, Policy);
  }
  llvm_unreachable("Unexpected attribute kind!");
}

