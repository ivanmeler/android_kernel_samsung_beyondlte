/*===- TableGen'erated file -------------------------------------*- C++ -*-===*\
|*                                                                            *|
|* Attribute classes' definitions                                             *|
|*                                                                            *|
|* Automatically generated file, do not edit!                                 *|
|*                                                                            *|
\*===----------------------------------------------------------------------===*/

#ifndef LLVM_CLANG_ATTR_CLASSES_INC
#define LLVM_CLANG_ATTR_CLASSES_INC

class AArch64VectorPcsAttr : public InheritableAttr {
public:
  static AArch64VectorPcsAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AArch64VectorPcsAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AArch64VectorPcsAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AArch64VectorPcsAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AArch64VectorPcsAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static AArch64VectorPcsAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  AArch64VectorPcsAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AArch64VectorPcs, false, false)
  {
  }

  AArch64VectorPcsAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::AArch64VectorPcs; }
};

class AMDGPUFlatWorkGroupSizeAttr : public InheritableAttr {
Expr * min;

Expr * max;

public:
  static AMDGPUFlatWorkGroupSizeAttr *CreateImplicit(ASTContext &Ctx, Expr * Min, Expr * Max, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AMDGPUFlatWorkGroupSizeAttr(Ctx, CommonInfo, Min, Max);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AMDGPUFlatWorkGroupSizeAttr *Create(ASTContext &Ctx, Expr * Min, Expr * Max, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AMDGPUFlatWorkGroupSizeAttr(Ctx, CommonInfo, Min, Max);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AMDGPUFlatWorkGroupSizeAttr *CreateImplicit(ASTContext &Ctx, Expr * Min, Expr * Max, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Min, Max, I);
  }
  static AMDGPUFlatWorkGroupSizeAttr *Create(ASTContext &Ctx, Expr * Min, Expr * Max, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Min, Max, I);
  }
  AMDGPUFlatWorkGroupSizeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Min
              , Expr * Max
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AMDGPUFlatWorkGroupSize, false, false)
              , min(Min)
              , max(Max)
  {
  }

  AMDGPUFlatWorkGroupSizeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getMin() const {
    return min;
  }

  Expr * getMax() const {
    return max;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AMDGPUFlatWorkGroupSize; }
};

class AMDGPUNumSGPRAttr : public InheritableAttr {
unsigned numSGPR;

public:
  static AMDGPUNumSGPRAttr *CreateImplicit(ASTContext &Ctx, unsigned NumSGPR, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AMDGPUNumSGPRAttr(Ctx, CommonInfo, NumSGPR);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AMDGPUNumSGPRAttr *Create(ASTContext &Ctx, unsigned NumSGPR, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AMDGPUNumSGPRAttr(Ctx, CommonInfo, NumSGPR);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AMDGPUNumSGPRAttr *CreateImplicit(ASTContext &Ctx, unsigned NumSGPR, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, NumSGPR, I);
  }
  static AMDGPUNumSGPRAttr *Create(ASTContext &Ctx, unsigned NumSGPR, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, NumSGPR, I);
  }
  AMDGPUNumSGPRAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned NumSGPR
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AMDGPUNumSGPR, false, false)
              , numSGPR(NumSGPR)
  {
  }

  AMDGPUNumSGPRAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getNumSGPR() const {
    return numSGPR;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AMDGPUNumSGPR; }
};

class AMDGPUNumVGPRAttr : public InheritableAttr {
unsigned numVGPR;

public:
  static AMDGPUNumVGPRAttr *CreateImplicit(ASTContext &Ctx, unsigned NumVGPR, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AMDGPUNumVGPRAttr(Ctx, CommonInfo, NumVGPR);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AMDGPUNumVGPRAttr *Create(ASTContext &Ctx, unsigned NumVGPR, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AMDGPUNumVGPRAttr(Ctx, CommonInfo, NumVGPR);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AMDGPUNumVGPRAttr *CreateImplicit(ASTContext &Ctx, unsigned NumVGPR, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, NumVGPR, I);
  }
  static AMDGPUNumVGPRAttr *Create(ASTContext &Ctx, unsigned NumVGPR, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, NumVGPR, I);
  }
  AMDGPUNumVGPRAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned NumVGPR
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AMDGPUNumVGPR, false, false)
              , numVGPR(NumVGPR)
  {
  }

  AMDGPUNumVGPRAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getNumVGPR() const {
    return numVGPR;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AMDGPUNumVGPR; }
};

class AMDGPUWavesPerEUAttr : public InheritableAttr {
Expr * min;

Expr * max;

public:
  static AMDGPUWavesPerEUAttr *CreateImplicit(ASTContext &Ctx, Expr * Min, Expr * Max, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AMDGPUWavesPerEUAttr(Ctx, CommonInfo, Min, Max);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AMDGPUWavesPerEUAttr *Create(ASTContext &Ctx, Expr * Min, Expr * Max, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AMDGPUWavesPerEUAttr(Ctx, CommonInfo, Min, Max);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AMDGPUWavesPerEUAttr *CreateImplicit(ASTContext &Ctx, Expr * Min, Expr * Max, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Min, Max, I);
  }
  static AMDGPUWavesPerEUAttr *Create(ASTContext &Ctx, Expr * Min, Expr * Max, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Min, Max, I);
  }
  AMDGPUWavesPerEUAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Min
              , Expr * Max
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AMDGPUWavesPerEU, false, false)
              , min(Min)
              , max(Max)
  {
  }

  AMDGPUWavesPerEUAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Min
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AMDGPUWavesPerEU, false, false)
              , min(Min)
              , max()
  {
  }

  AMDGPUWavesPerEUAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getMin() const {
    return min;
  }

  Expr * getMax() const {
    return max;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AMDGPUWavesPerEU; }
};

class ARMInterruptAttr : public InheritableAttr {
public:
  enum InterruptType {
    IRQ,
    FIQ,
    SWI,
    ABORT,
    UNDEF,
    Generic
  };
private:
  InterruptType interrupt;

public:
  static ARMInterruptAttr *CreateImplicit(ASTContext &Ctx, InterruptType Interrupt, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ARMInterruptAttr(Ctx, CommonInfo, Interrupt);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ARMInterruptAttr *Create(ASTContext &Ctx, InterruptType Interrupt, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ARMInterruptAttr(Ctx, CommonInfo, Interrupt);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ARMInterruptAttr *CreateImplicit(ASTContext &Ctx, InterruptType Interrupt, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Interrupt, I);
  }
  static ARMInterruptAttr *Create(ASTContext &Ctx, InterruptType Interrupt, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Interrupt, I);
  }
  ARMInterruptAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , InterruptType Interrupt
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ARMInterrupt, false, false)
              , interrupt(Interrupt)
  {
  }

  ARMInterruptAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ARMInterrupt, false, false)
              , interrupt(InterruptType(0))
  {
  }

  ARMInterruptAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  InterruptType getInterrupt() const {
    return interrupt;
  }

  static bool ConvertStrToInterruptType(StringRef Val, InterruptType &Out) {
    Optional<InterruptType> R = llvm::StringSwitch<Optional<InterruptType>>(Val)
      .Case("IRQ", ARMInterruptAttr::IRQ)
      .Case("FIQ", ARMInterruptAttr::FIQ)
      .Case("SWI", ARMInterruptAttr::SWI)
      .Case("ABORT", ARMInterruptAttr::ABORT)
      .Case("UNDEF", ARMInterruptAttr::UNDEF)
      .Case("", ARMInterruptAttr::Generic)
      .Default(Optional<InterruptType>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertInterruptTypeToStr(InterruptType Val) {
    switch(Val) {
    case ARMInterruptAttr::IRQ: return "IRQ";
    case ARMInterruptAttr::FIQ: return "FIQ";
    case ARMInterruptAttr::SWI: return "SWI";
    case ARMInterruptAttr::ABORT: return "ABORT";
    case ARMInterruptAttr::UNDEF: return "UNDEF";
    case ARMInterruptAttr::Generic: return "";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::ARMInterrupt; }
};

class AVRInterruptAttr : public InheritableAttr {
public:
  static AVRInterruptAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AVRInterruptAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AVRInterruptAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AVRInterruptAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AVRInterruptAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static AVRInterruptAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  AVRInterruptAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AVRInterrupt, false, false)
  {
  }

  AVRInterruptAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::AVRInterrupt; }
};

class AVRSignalAttr : public InheritableAttr {
public:
  static AVRSignalAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AVRSignalAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AVRSignalAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AVRSignalAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AVRSignalAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static AVRSignalAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  AVRSignalAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AVRSignal, false, false)
  {
  }

  AVRSignalAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::AVRSignal; }
};

class AbiTagAttr : public Attr {
  unsigned tags_Size;
  StringRef *tags_;

public:
  static AbiTagAttr *CreateImplicit(ASTContext &Ctx, StringRef *Tags, unsigned TagsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AbiTagAttr(Ctx, CommonInfo, Tags, TagsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AbiTagAttr *Create(ASTContext &Ctx, StringRef *Tags, unsigned TagsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AbiTagAttr(Ctx, CommonInfo, Tags, TagsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AbiTagAttr *CreateImplicit(ASTContext &Ctx, StringRef *Tags, unsigned TagsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Tags, TagsSize, I);
  }
  static AbiTagAttr *Create(ASTContext &Ctx, StringRef *Tags, unsigned TagsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Tags, TagsSize, I);
  }
  AbiTagAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , StringRef *Tags, unsigned TagsSize
             )
    : Attr(Ctx, CommonInfo, attr::AbiTag, false)
              , tags_Size(TagsSize), tags_(new (Ctx, 16) StringRef[tags_Size])
  {
    for (size_t I = 0, E = tags_Size; I != E;
         ++I) {
      StringRef Ref = Tags[I];
      if (!Ref.empty()) {
        char *Mem = new (Ctx, 1) char[Ref.size()];
        std::memcpy(Mem, Ref.data(), Ref.size());
        tags_[I] = StringRef(Mem, Ref.size());
      }
    }
  }

  AbiTagAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::AbiTag, false)
              , tags_Size(0), tags_(nullptr)
  {
  }

  AbiTagAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef StringRef* tags_iterator;
  tags_iterator tags_begin() const { return tags_; }
  tags_iterator tags_end() const { return tags_ + tags_Size; }
  unsigned tags_size() const { return tags_Size; }
  llvm::iterator_range<tags_iterator> tags() const { return llvm::make_range(tags_begin(), tags_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::AbiTag; }
};

class AcquireCapabilityAttr : public InheritableAttr {
  unsigned args_Size;
  Expr * *args_;

public:
  enum Spelling {
    GNU_acquire_capability = 0,
    CXX11_clang_acquire_capability = 1,
    GNU_acquire_shared_capability = 2,
    CXX11_clang_acquire_shared_capability = 3,
    GNU_exclusive_lock_function = 4,
    GNU_shared_lock_function = 5,
  SpellingNotCalculated = 15

  };

  static AcquireCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AcquireCapabilityAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AcquireCapabilityAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AcquireCapabilityAttr(Ctx, CommonInfo, Args, ArgsSize);
    return A;
  }

  static AcquireCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, AcquireCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static AcquireCapabilityAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, AcquireCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Args, ArgsSize, I);
  }
  AcquireCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AcquireCapability, true, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  AcquireCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AcquireCapability, true, true)
              , args_Size(0), args_(nullptr)
  {
  }

  AcquireCapabilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_acquire_capability;
    case 1: return CXX11_clang_acquire_capability;
    case 2: return GNU_acquire_shared_capability;
    case 3: return CXX11_clang_acquire_shared_capability;
    case 4: return GNU_exclusive_lock_function;
    case 5: return GNU_shared_lock_function;
  }
  }
  bool isShared() const { return getAttributeSpellingListIndex() == 2 ||
    getAttributeSpellingListIndex() == 3 ||
    getAttributeSpellingListIndex() == 5; }
  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::AcquireCapability; }
};

class AcquireHandleAttr : public InheritableAttr {
unsigned handleTypeLength;
char *handleType;

public:
  static AcquireHandleAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef HandleType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AcquireHandleAttr(Ctx, CommonInfo, HandleType);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AcquireHandleAttr *Create(ASTContext &Ctx, llvm::StringRef HandleType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AcquireHandleAttr(Ctx, CommonInfo, HandleType);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AcquireHandleAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef HandleType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, HandleType, I);
  }
  static AcquireHandleAttr *Create(ASTContext &Ctx, llvm::StringRef HandleType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, HandleType, I);
  }
  AcquireHandleAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef HandleType
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AcquireHandle, false, false)
              , handleTypeLength(HandleType.size()),handleType(new (Ctx, 1) char[handleTypeLength])
  {
      if (!HandleType.empty())
        std::memcpy(handleType, HandleType.data(), handleTypeLength);
  }

  AcquireHandleAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getHandleType() const {
    return llvm::StringRef(handleType, handleTypeLength);
  }
  unsigned getHandleTypeLength() const {
    return handleTypeLength;
  }
  void setHandleType(ASTContext &C, llvm::StringRef S) {
    handleTypeLength = S.size();
    this->handleType = new (C, 1) char [handleTypeLength];
    if (!S.empty())
      std::memcpy(this->handleType, S.data(), handleTypeLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AcquireHandle; }
};

class AcquiredAfterAttr : public InheritableAttr {
  unsigned args_Size;
  Expr * *args_;

public:
  static AcquiredAfterAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AcquiredAfterAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AcquiredAfterAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AcquiredAfterAttr(Ctx, CommonInfo, Args, ArgsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AcquiredAfterAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static AcquiredAfterAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Args, ArgsSize, I);
  }
  AcquiredAfterAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AcquiredAfter, true, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  AcquiredAfterAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AcquiredAfter, true, true)
              , args_Size(0), args_(nullptr)
  {
  }

  AcquiredAfterAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::AcquiredAfter; }
};

class AcquiredBeforeAttr : public InheritableAttr {
  unsigned args_Size;
  Expr * *args_;

public:
  static AcquiredBeforeAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AcquiredBeforeAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AcquiredBeforeAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AcquiredBeforeAttr(Ctx, CommonInfo, Args, ArgsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AcquiredBeforeAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static AcquiredBeforeAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Args, ArgsSize, I);
  }
  AcquiredBeforeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AcquiredBefore, true, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  AcquiredBeforeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AcquiredBefore, true, true)
              , args_Size(0), args_(nullptr)
  {
  }

  AcquiredBeforeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::AcquiredBefore; }
};

class AddressSpaceAttr : public TypeAttr {
int addressSpace;

public:
  static AddressSpaceAttr *CreateImplicit(ASTContext &Ctx, int AddressSpace, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AddressSpaceAttr(Ctx, CommonInfo, AddressSpace);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AddressSpaceAttr *Create(ASTContext &Ctx, int AddressSpace, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AddressSpaceAttr(Ctx, CommonInfo, AddressSpace);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AddressSpaceAttr *CreateImplicit(ASTContext &Ctx, int AddressSpace, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, AddressSpace, I);
  }
  static AddressSpaceAttr *Create(ASTContext &Ctx, int AddressSpace, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, AddressSpace, I);
  }
  AddressSpaceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , int AddressSpace
             )
    : TypeAttr(Ctx, CommonInfo, attr::AddressSpace, false)
              , addressSpace(AddressSpace)
  {
  }

  AddressSpaceAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  int getAddressSpace() const {
    return addressSpace;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AddressSpace; }
};

class AliasAttr : public Attr {
unsigned aliaseeLength;
char *aliasee;

public:
  static AliasAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Aliasee, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AliasAttr(Ctx, CommonInfo, Aliasee);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AliasAttr *Create(ASTContext &Ctx, llvm::StringRef Aliasee, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AliasAttr(Ctx, CommonInfo, Aliasee);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AliasAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Aliasee, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Aliasee, I);
  }
  static AliasAttr *Create(ASTContext &Ctx, llvm::StringRef Aliasee, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Aliasee, I);
  }
  AliasAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Aliasee
             )
    : Attr(Ctx, CommonInfo, attr::Alias, false)
              , aliaseeLength(Aliasee.size()),aliasee(new (Ctx, 1) char[aliaseeLength])
  {
      if (!Aliasee.empty())
        std::memcpy(aliasee, Aliasee.data(), aliaseeLength);
  }

  AliasAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getAliasee() const {
    return llvm::StringRef(aliasee, aliaseeLength);
  }
  unsigned getAliaseeLength() const {
    return aliaseeLength;
  }
  void setAliasee(ASTContext &C, llvm::StringRef S) {
    aliaseeLength = S.size();
    this->aliasee = new (C, 1) char [aliaseeLength];
    if (!S.empty())
      std::memcpy(this->aliasee, S.data(), aliaseeLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Alias; }
};

class AlignMac68kAttr : public InheritableAttr {
public:
  static AlignMac68kAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlignMac68kAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AlignMac68kAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlignMac68kAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AlignMac68kAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static AlignMac68kAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  AlignMac68kAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AlignMac68k, false, false)
  {
  }

  AlignMac68kAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::AlignMac68k; }
};

class AlignValueAttr : public Attr {
Expr * alignment;

public:
  static AlignValueAttr *CreateImplicit(ASTContext &Ctx, Expr * Alignment, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlignValueAttr(Ctx, CommonInfo, Alignment);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AlignValueAttr *Create(ASTContext &Ctx, Expr * Alignment, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlignValueAttr(Ctx, CommonInfo, Alignment);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AlignValueAttr *CreateImplicit(ASTContext &Ctx, Expr * Alignment, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Alignment, I);
  }
  static AlignValueAttr *Create(ASTContext &Ctx, Expr * Alignment, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Alignment, I);
  }
  AlignValueAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Alignment
             )
    : Attr(Ctx, CommonInfo, attr::AlignValue, false)
              , alignment(Alignment)
  {
  }

  AlignValueAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getAlignment() const {
    return alignment;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AlignValue; }
};

class AlignedAttr : public InheritableAttr {
bool isalignmentExpr;
union {
Expr *alignmentExpr;
TypeSourceInfo *alignmentType;
};

public:
  enum Spelling {
    GNU_aligned = 0,
    CXX11_gnu_aligned = 1,
    Declspec_align = 2,
    Keyword_alignas = 3,
    Keyword_Alignas = 4,
  SpellingNotCalculated = 15

  };

  static AlignedAttr *CreateImplicit(ASTContext &Ctx, bool IsAlignmentExpr, void *Alignment, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlignedAttr(Ctx, CommonInfo, IsAlignmentExpr, Alignment);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AlignedAttr *Create(ASTContext &Ctx, bool IsAlignmentExpr, void *Alignment, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlignedAttr(Ctx, CommonInfo, IsAlignmentExpr, Alignment);
    return A;
  }

  static AlignedAttr *CreateImplicit(ASTContext &Ctx, bool IsAlignmentExpr, void *Alignment, SourceRange Range, AttributeCommonInfo::Syntax Syntax, AlignedAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, IsAlignmentExpr, Alignment, I);
  }
  static AlignedAttr *Create(ASTContext &Ctx, bool IsAlignmentExpr, void *Alignment, SourceRange Range, AttributeCommonInfo::Syntax Syntax, AlignedAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, IsAlignmentExpr, Alignment, I);
  }
  AlignedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , bool IsAlignmentExpr, void *Alignment
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Aligned, false, false)
              , isalignmentExpr(IsAlignmentExpr)
  {
    if (isalignmentExpr)
       alignmentExpr = reinterpret_cast<Expr *>(Alignment);
    else
       alignmentType = reinterpret_cast<TypeSourceInfo *>(Alignment);
  }

  AlignedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Aligned, false, false)
              , isalignmentExpr(false)
  {
  }

  AlignedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_aligned;
    case 1: return CXX11_gnu_aligned;
    case 2: return Declspec_align;
    case 3: return Keyword_alignas;
    case 4: return Keyword_Alignas;
  }
  }
  bool isGNU() const { return getAttributeSpellingListIndex() == 0 ||
    getAttributeSpellingListIndex() == 1; }
  bool isC11() const { return getAttributeSpellingListIndex() == 4; }
  bool isAlignas() const { return getAttributeSpellingListIndex() == 3 ||
    getAttributeSpellingListIndex() == 4; }
  bool isDeclspec() const { return getAttributeSpellingListIndex() == 2; }
  bool isAlignmentDependent() const;
  unsigned getAlignment(ASTContext &Ctx) const;
  bool isAlignmentExpr() const {
    return isalignmentExpr;
  }
  Expr *getAlignmentExpr() const {
    assert(isalignmentExpr);
    return alignmentExpr;
  }
  TypeSourceInfo *getAlignmentType() const {
    assert(!isalignmentExpr);
    return alignmentType;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Aligned; }
};

class AllocAlignAttr : public InheritableAttr {
ParamIdx paramIndex;

public:
  static AllocAlignAttr *CreateImplicit(ASTContext &Ctx, ParamIdx ParamIndex, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AllocAlignAttr(Ctx, CommonInfo, ParamIndex);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AllocAlignAttr *Create(ASTContext &Ctx, ParamIdx ParamIndex, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AllocAlignAttr(Ctx, CommonInfo, ParamIndex);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AllocAlignAttr *CreateImplicit(ASTContext &Ctx, ParamIdx ParamIndex, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, ParamIndex, I);
  }
  static AllocAlignAttr *Create(ASTContext &Ctx, ParamIdx ParamIndex, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, ParamIndex, I);
  }
  AllocAlignAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ParamIdx ParamIndex
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AllocAlign, false, false)
              , paramIndex(ParamIndex)
  {
  }

  AllocAlignAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  ParamIdx getParamIndex() const {
    return paramIndex;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AllocAlign; }
};

class AllocSizeAttr : public InheritableAttr {
ParamIdx elemSizeParam;

ParamIdx numElemsParam;

public:
  static AllocSizeAttr *CreateImplicit(ASTContext &Ctx, ParamIdx ElemSizeParam, ParamIdx NumElemsParam, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AllocSizeAttr(Ctx, CommonInfo, ElemSizeParam, NumElemsParam);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AllocSizeAttr *Create(ASTContext &Ctx, ParamIdx ElemSizeParam, ParamIdx NumElemsParam, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AllocSizeAttr(Ctx, CommonInfo, ElemSizeParam, NumElemsParam);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AllocSizeAttr *CreateImplicit(ASTContext &Ctx, ParamIdx ElemSizeParam, ParamIdx NumElemsParam, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, ElemSizeParam, NumElemsParam, I);
  }
  static AllocSizeAttr *Create(ASTContext &Ctx, ParamIdx ElemSizeParam, ParamIdx NumElemsParam, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, ElemSizeParam, NumElemsParam, I);
  }
  AllocSizeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ParamIdx ElemSizeParam
              , ParamIdx NumElemsParam
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AllocSize, false, false)
              , elemSizeParam(ElemSizeParam)
              , numElemsParam(NumElemsParam)
  {
  }

  AllocSizeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ParamIdx ElemSizeParam
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AllocSize, false, false)
              , elemSizeParam(ElemSizeParam)
              , numElemsParam()
  {
  }

  AllocSizeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  ParamIdx getElemSizeParam() const {
    return elemSizeParam;
  }

  ParamIdx getNumElemsParam() const {
    return numElemsParam;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AllocSize; }
};

class AlwaysDestroyAttr : public InheritableAttr {
public:
  static AlwaysDestroyAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlwaysDestroyAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AlwaysDestroyAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlwaysDestroyAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AlwaysDestroyAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static AlwaysDestroyAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  AlwaysDestroyAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AlwaysDestroy, false, false)
  {
  }

  AlwaysDestroyAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::AlwaysDestroy; }
};

class AlwaysInlineAttr : public InheritableAttr {
public:
  enum Spelling {
    GNU_always_inline = 0,
    CXX11_gnu_always_inline = 1,
    Keyword_forceinline = 2,
  SpellingNotCalculated = 15

  };

  static AlwaysInlineAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlwaysInlineAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AlwaysInlineAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AlwaysInlineAttr(Ctx, CommonInfo);
    return A;
  }

  static AlwaysInlineAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, AlwaysInlineAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static AlwaysInlineAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, AlwaysInlineAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  AlwaysInlineAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AlwaysInline, false, false)
  {
  }

  AlwaysInlineAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_always_inline;
    case 1: return CXX11_gnu_always_inline;
    case 2: return Keyword_forceinline;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::AlwaysInline; }
};

class AnalyzerNoReturnAttr : public InheritableAttr {
public:
  static AnalyzerNoReturnAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnalyzerNoReturnAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnalyzerNoReturnAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnalyzerNoReturnAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnalyzerNoReturnAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static AnalyzerNoReturnAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  AnalyzerNoReturnAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AnalyzerNoReturn, false, false)
  {
  }

  AnalyzerNoReturnAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::AnalyzerNoReturn; }
};

class AnnotateAttr : public InheritableParamAttr {
unsigned annotationLength;
char *annotation;

public:
  static AnnotateAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Annotation, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnnotateAttr(Ctx, CommonInfo, Annotation);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnnotateAttr *Create(ASTContext &Ctx, llvm::StringRef Annotation, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnnotateAttr(Ctx, CommonInfo, Annotation);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnnotateAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Annotation, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Annotation, I);
  }
  static AnnotateAttr *Create(ASTContext &Ctx, llvm::StringRef Annotation, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Annotation, I);
  }
  AnnotateAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Annotation
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::Annotate, false, false)
              , annotationLength(Annotation.size()),annotation(new (Ctx, 1) char[annotationLength])
  {
      if (!Annotation.empty())
        std::memcpy(annotation, Annotation.data(), annotationLength);
  }

  AnnotateAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getAnnotation() const {
    return llvm::StringRef(annotation, annotationLength);
  }
  unsigned getAnnotationLength() const {
    return annotationLength;
  }
  void setAnnotation(ASTContext &C, llvm::StringRef S) {
    annotationLength = S.size();
    this->annotation = new (C, 1) char [annotationLength];
    if (!S.empty())
      std::memcpy(this->annotation, S.data(), annotationLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Annotate; }
};

class AnyX86InterruptAttr : public InheritableAttr {
public:
  static AnyX86InterruptAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnyX86InterruptAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnyX86InterruptAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnyX86InterruptAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnyX86InterruptAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static AnyX86InterruptAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  AnyX86InterruptAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AnyX86Interrupt, false, false)
  {
  }

  AnyX86InterruptAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::AnyX86Interrupt; }
};

class AnyX86NoCallerSavedRegistersAttr : public InheritableAttr {
public:
  static AnyX86NoCallerSavedRegistersAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnyX86NoCallerSavedRegistersAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnyX86NoCallerSavedRegistersAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnyX86NoCallerSavedRegistersAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnyX86NoCallerSavedRegistersAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static AnyX86NoCallerSavedRegistersAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  AnyX86NoCallerSavedRegistersAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AnyX86NoCallerSavedRegisters, false, false)
  {
  }

  AnyX86NoCallerSavedRegistersAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::AnyX86NoCallerSavedRegisters; }
};

class AnyX86NoCfCheckAttr : public InheritableAttr {
public:
  static AnyX86NoCfCheckAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnyX86NoCfCheckAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnyX86NoCfCheckAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AnyX86NoCfCheckAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AnyX86NoCfCheckAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static AnyX86NoCfCheckAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  AnyX86NoCfCheckAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AnyX86NoCfCheck, false, false)
  {
  }

  AnyX86NoCfCheckAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::AnyX86NoCfCheck; }
};

class ArcWeakrefUnavailableAttr : public InheritableAttr {
public:
  static ArcWeakrefUnavailableAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArcWeakrefUnavailableAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArcWeakrefUnavailableAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArcWeakrefUnavailableAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArcWeakrefUnavailableAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ArcWeakrefUnavailableAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ArcWeakrefUnavailableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ArcWeakrefUnavailable, false, false)
  {
  }

  ArcWeakrefUnavailableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ArcWeakrefUnavailable; }
};

class ArgumentWithTypeTagAttr : public InheritableAttr {
IdentifierInfo * argumentKind;

ParamIdx argumentIdx;

ParamIdx typeTagIdx;

bool isPointer;

public:
  enum Spelling {
    GNU_argument_with_type_tag = 0,
    CXX11_clang_argument_with_type_tag = 1,
    C2x_clang_argument_with_type_tag = 2,
    GNU_pointer_with_type_tag = 3,
    CXX11_clang_pointer_with_type_tag = 4,
    C2x_clang_pointer_with_type_tag = 5,
  SpellingNotCalculated = 15

  };

  static ArgumentWithTypeTagAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * ArgumentKind, ParamIdx ArgumentIdx, ParamIdx TypeTagIdx, bool IsPointer, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArgumentWithTypeTagAttr(Ctx, CommonInfo, ArgumentKind, ArgumentIdx, TypeTagIdx, IsPointer);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArgumentWithTypeTagAttr *Create(ASTContext &Ctx, IdentifierInfo * ArgumentKind, ParamIdx ArgumentIdx, ParamIdx TypeTagIdx, bool IsPointer, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArgumentWithTypeTagAttr(Ctx, CommonInfo, ArgumentKind, ArgumentIdx, TypeTagIdx, IsPointer);
    return A;
  }

  static ArgumentWithTypeTagAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * ArgumentKind, ParamIdx ArgumentIdx, ParamIdx TypeTagIdx, bool IsPointer, SourceRange Range, AttributeCommonInfo::Syntax Syntax, ArgumentWithTypeTagAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, ArgumentKind, ArgumentIdx, TypeTagIdx, IsPointer, I);
  }
  static ArgumentWithTypeTagAttr *Create(ASTContext &Ctx, IdentifierInfo * ArgumentKind, ParamIdx ArgumentIdx, ParamIdx TypeTagIdx, bool IsPointer, SourceRange Range, AttributeCommonInfo::Syntax Syntax, ArgumentWithTypeTagAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, ArgumentKind, ArgumentIdx, TypeTagIdx, IsPointer, I);
  }
  static ArgumentWithTypeTagAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * ArgumentKind, ParamIdx ArgumentIdx, ParamIdx TypeTagIdx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArgumentWithTypeTagAttr(Ctx, CommonInfo, ArgumentKind, ArgumentIdx, TypeTagIdx);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArgumentWithTypeTagAttr *Create(ASTContext &Ctx, IdentifierInfo * ArgumentKind, ParamIdx ArgumentIdx, ParamIdx TypeTagIdx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArgumentWithTypeTagAttr(Ctx, CommonInfo, ArgumentKind, ArgumentIdx, TypeTagIdx);
    return A;
  }

  static ArgumentWithTypeTagAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * ArgumentKind, ParamIdx ArgumentIdx, ParamIdx TypeTagIdx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, ArgumentWithTypeTagAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, ArgumentKind, ArgumentIdx, TypeTagIdx, I);
  }
  static ArgumentWithTypeTagAttr *Create(ASTContext &Ctx, IdentifierInfo * ArgumentKind, ParamIdx ArgumentIdx, ParamIdx TypeTagIdx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, ArgumentWithTypeTagAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, ArgumentKind, ArgumentIdx, TypeTagIdx, I);
  }
  ArgumentWithTypeTagAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * ArgumentKind
              , ParamIdx ArgumentIdx
              , ParamIdx TypeTagIdx
              , bool IsPointer
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ArgumentWithTypeTag, false, false)
              , argumentKind(ArgumentKind)
              , argumentIdx(ArgumentIdx)
              , typeTagIdx(TypeTagIdx)
              , isPointer(IsPointer)
  {
  }

  ArgumentWithTypeTagAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * ArgumentKind
              , ParamIdx ArgumentIdx
              , ParamIdx TypeTagIdx
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ArgumentWithTypeTag, false, false)
              , argumentKind(ArgumentKind)
              , argumentIdx(ArgumentIdx)
              , typeTagIdx(TypeTagIdx)
              , isPointer()
  {
  }

  ArgumentWithTypeTagAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_argument_with_type_tag;
    case 1: return CXX11_clang_argument_with_type_tag;
    case 2: return C2x_clang_argument_with_type_tag;
    case 3: return GNU_pointer_with_type_tag;
    case 4: return CXX11_clang_pointer_with_type_tag;
    case 5: return C2x_clang_pointer_with_type_tag;
  }
  }
  IdentifierInfo * getArgumentKind() const {
    return argumentKind;
  }

  ParamIdx getArgumentIdx() const {
    return argumentIdx;
  }

  ParamIdx getTypeTagIdx() const {
    return typeTagIdx;
  }

  bool getIsPointer() const {
    return isPointer;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ArgumentWithTypeTag; }
};

class ArmMveAliasAttr : public InheritableAttr {
IdentifierInfo * builtinName;

public:
  static ArmMveAliasAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * BuiltinName, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArmMveAliasAttr(Ctx, CommonInfo, BuiltinName);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArmMveAliasAttr *Create(ASTContext &Ctx, IdentifierInfo * BuiltinName, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArmMveAliasAttr(Ctx, CommonInfo, BuiltinName);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArmMveAliasAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * BuiltinName, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, BuiltinName, I);
  }
  static ArmMveAliasAttr *Create(ASTContext &Ctx, IdentifierInfo * BuiltinName, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, BuiltinName, I);
  }
  ArmMveAliasAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * BuiltinName
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ArmMveAlias, false, false)
              , builtinName(BuiltinName)
  {
  }

  ArmMveAliasAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getBuiltinName() const {
    return builtinName;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ArmMveAlias; }
};

class ArmMveStrictPolymorphismAttr : public TypeAttr {
public:
  static ArmMveStrictPolymorphismAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArmMveStrictPolymorphismAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArmMveStrictPolymorphismAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArmMveStrictPolymorphismAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArmMveStrictPolymorphismAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ArmMveStrictPolymorphismAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ArmMveStrictPolymorphismAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::ArmMveStrictPolymorphism, false)
  {
  }

  ArmMveStrictPolymorphismAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ArmMveStrictPolymorphism; }
};

class ArtificialAttr : public InheritableAttr {
public:
  static ArtificialAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArtificialAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArtificialAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ArtificialAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ArtificialAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ArtificialAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ArtificialAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Artificial, false, false)
  {
  }

  ArtificialAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Artificial; }
};

class AsmLabelAttr : public InheritableAttr {
unsigned labelLength;
char *label;

bool isLiteralLabel;

public:
  static AsmLabelAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Label, bool IsLiteralLabel, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AsmLabelAttr(Ctx, CommonInfo, Label, IsLiteralLabel);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AsmLabelAttr *Create(ASTContext &Ctx, llvm::StringRef Label, bool IsLiteralLabel, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AsmLabelAttr(Ctx, CommonInfo, Label, IsLiteralLabel);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AsmLabelAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Label, bool IsLiteralLabel, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Label, IsLiteralLabel, I);
  }
  static AsmLabelAttr *Create(ASTContext &Ctx, llvm::StringRef Label, bool IsLiteralLabel, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Label, IsLiteralLabel, I);
  }
  static AsmLabelAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Label, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AsmLabelAttr(Ctx, CommonInfo, Label);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AsmLabelAttr *Create(ASTContext &Ctx, llvm::StringRef Label, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AsmLabelAttr(Ctx, CommonInfo, Label);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AsmLabelAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Label, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Label, I);
  }
  static AsmLabelAttr *Create(ASTContext &Ctx, llvm::StringRef Label, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Label, I);
  }
  AsmLabelAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Label
              , bool IsLiteralLabel
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AsmLabel, false, false)
              , labelLength(Label.size()),label(new (Ctx, 1) char[labelLength])
              , isLiteralLabel(IsLiteralLabel)
  {
      if (!Label.empty())
        std::memcpy(label, Label.data(), labelLength);
  }

  AsmLabelAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Label
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AsmLabel, false, false)
              , labelLength(Label.size()),label(new (Ctx, 1) char[labelLength])
              , isLiteralLabel()
  {
      if (!Label.empty())
        std::memcpy(label, Label.data(), labelLength);
  }

  AsmLabelAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getLabel() const {
    return llvm::StringRef(label, labelLength);
  }
  unsigned getLabelLength() const {
    return labelLength;
  }
  void setLabel(ASTContext &C, llvm::StringRef S) {
    labelLength = S.size();
    this->label = new (C, 1) char [labelLength];
    if (!S.empty())
      std::memcpy(this->label, S.data(), labelLength);
  }

  bool getIsLiteralLabel() const {
    return isLiteralLabel;
  }


bool isEquivalent(AsmLabelAttr *Other) const {
  return getLabel() == Other->getLabel() && getIsLiteralLabel() == Other->getIsLiteralLabel();
}


  static bool classof(const Attr *A) { return A->getKind() == attr::AsmLabel; }
};

class AssertCapabilityAttr : public InheritableAttr {
  unsigned args_Size;
  Expr * *args_;

public:
  enum Spelling {
    GNU_assert_capability = 0,
    CXX11_clang_assert_capability = 1,
    GNU_assert_shared_capability = 2,
    CXX11_clang_assert_shared_capability = 3,
  SpellingNotCalculated = 15

  };

  static AssertCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AssertCapabilityAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AssertCapabilityAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AssertCapabilityAttr(Ctx, CommonInfo, Args, ArgsSize);
    return A;
  }

  static AssertCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, AssertCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static AssertCapabilityAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, AssertCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Args, ArgsSize, I);
  }
  AssertCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AssertCapability, true, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  AssertCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AssertCapability, true, true)
              , args_Size(0), args_(nullptr)
  {
  }

  AssertCapabilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_assert_capability;
    case 1: return CXX11_clang_assert_capability;
    case 2: return GNU_assert_shared_capability;
    case 3: return CXX11_clang_assert_shared_capability;
  }
  }
  bool isShared() const { return getAttributeSpellingListIndex() == 2 ||
    getAttributeSpellingListIndex() == 3; }
  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::AssertCapability; }
};

class AssertExclusiveLockAttr : public InheritableAttr {
  unsigned args_Size;
  Expr * *args_;

public:
  static AssertExclusiveLockAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AssertExclusiveLockAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AssertExclusiveLockAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AssertExclusiveLockAttr(Ctx, CommonInfo, Args, ArgsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AssertExclusiveLockAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static AssertExclusiveLockAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Args, ArgsSize, I);
  }
  AssertExclusiveLockAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AssertExclusiveLock, true, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  AssertExclusiveLockAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AssertExclusiveLock, true, true)
              , args_Size(0), args_(nullptr)
  {
  }

  AssertExclusiveLockAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::AssertExclusiveLock; }
};

class AssertSharedLockAttr : public InheritableAttr {
  unsigned args_Size;
  Expr * *args_;

public:
  static AssertSharedLockAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AssertSharedLockAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AssertSharedLockAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AssertSharedLockAttr(Ctx, CommonInfo, Args, ArgsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AssertSharedLockAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static AssertSharedLockAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Args, ArgsSize, I);
  }
  AssertSharedLockAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AssertSharedLock, true, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  AssertSharedLockAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AssertSharedLock, true, true)
              , args_Size(0), args_(nullptr)
  {
  }

  AssertSharedLockAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::AssertSharedLock; }
};

class AssumeAlignedAttr : public InheritableAttr {
Expr * alignment;

Expr * offset;

public:
  static AssumeAlignedAttr *CreateImplicit(ASTContext &Ctx, Expr * Alignment, Expr * Offset, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AssumeAlignedAttr(Ctx, CommonInfo, Alignment, Offset);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AssumeAlignedAttr *Create(ASTContext &Ctx, Expr * Alignment, Expr * Offset, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AssumeAlignedAttr(Ctx, CommonInfo, Alignment, Offset);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AssumeAlignedAttr *CreateImplicit(ASTContext &Ctx, Expr * Alignment, Expr * Offset, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Alignment, Offset, I);
  }
  static AssumeAlignedAttr *Create(ASTContext &Ctx, Expr * Alignment, Expr * Offset, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Alignment, Offset, I);
  }
  AssumeAlignedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Alignment
              , Expr * Offset
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AssumeAligned, false, false)
              , alignment(Alignment)
              , offset(Offset)
  {
  }

  AssumeAlignedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Alignment
             )
    : InheritableAttr(Ctx, CommonInfo, attr::AssumeAligned, false, false)
              , alignment(Alignment)
              , offset()
  {
  }

  AssumeAlignedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getAlignment() const {
    return alignment;
  }

  Expr * getOffset() const {
    return offset;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::AssumeAligned; }
};

class AvailabilityAttr : public InheritableAttr {
IdentifierInfo * platform;

VersionTuple introduced;


VersionTuple deprecated;


VersionTuple obsoleted;


bool unavailable;

unsigned messageLength;
char *message;

bool strict;

unsigned replacementLength;
char *replacement;

int priority;

public:
  static AvailabilityAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Platform, VersionTuple Introduced, VersionTuple Deprecated, VersionTuple Obsoleted, bool Unavailable, llvm::StringRef Message, bool Strict, llvm::StringRef Replacement, int Priority, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AvailabilityAttr(Ctx, CommonInfo, Platform, Introduced, Deprecated, Obsoleted, Unavailable, Message, Strict, Replacement, Priority);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AvailabilityAttr *Create(ASTContext &Ctx, IdentifierInfo * Platform, VersionTuple Introduced, VersionTuple Deprecated, VersionTuple Obsoleted, bool Unavailable, llvm::StringRef Message, bool Strict, llvm::StringRef Replacement, int Priority, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) AvailabilityAttr(Ctx, CommonInfo, Platform, Introduced, Deprecated, Obsoleted, Unavailable, Message, Strict, Replacement, Priority);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static AvailabilityAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Platform, VersionTuple Introduced, VersionTuple Deprecated, VersionTuple Obsoleted, bool Unavailable, llvm::StringRef Message, bool Strict, llvm::StringRef Replacement, int Priority, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Platform, Introduced, Deprecated, Obsoleted, Unavailable, Message, Strict, Replacement, Priority, I);
  }
  static AvailabilityAttr *Create(ASTContext &Ctx, IdentifierInfo * Platform, VersionTuple Introduced, VersionTuple Deprecated, VersionTuple Obsoleted, bool Unavailable, llvm::StringRef Message, bool Strict, llvm::StringRef Replacement, int Priority, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Platform, Introduced, Deprecated, Obsoleted, Unavailable, Message, Strict, Replacement, Priority, I);
  }
  AvailabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * Platform
              , VersionTuple Introduced
              , VersionTuple Deprecated
              , VersionTuple Obsoleted
              , bool Unavailable
              , llvm::StringRef Message
              , bool Strict
              , llvm::StringRef Replacement
              , int Priority
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Availability, false, true)
              , platform(Platform)
              , introduced(Introduced)
              , deprecated(Deprecated)
              , obsoleted(Obsoleted)
              , unavailable(Unavailable)
              , messageLength(Message.size()),message(new (Ctx, 1) char[messageLength])
              , strict(Strict)
              , replacementLength(Replacement.size()),replacement(new (Ctx, 1) char[replacementLength])
              , priority(Priority)
  {
      if (!Message.empty())
        std::memcpy(message, Message.data(), messageLength);
      if (!Replacement.empty())
        std::memcpy(replacement, Replacement.data(), replacementLength);
  }

  AvailabilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getPlatform() const {
    return platform;
  }

  VersionTuple getIntroduced() const {
    return introduced;
  }
  void setIntroduced(ASTContext &C, VersionTuple V) {
    introduced = V;
  }

  VersionTuple getDeprecated() const {
    return deprecated;
  }
  void setDeprecated(ASTContext &C, VersionTuple V) {
    deprecated = V;
  }

  VersionTuple getObsoleted() const {
    return obsoleted;
  }
  void setObsoleted(ASTContext &C, VersionTuple V) {
    obsoleted = V;
  }

  bool getUnavailable() const {
    return unavailable;
  }

  llvm::StringRef getMessage() const {
    return llvm::StringRef(message, messageLength);
  }
  unsigned getMessageLength() const {
    return messageLength;
  }
  void setMessage(ASTContext &C, llvm::StringRef S) {
    messageLength = S.size();
    this->message = new (C, 1) char [messageLength];
    if (!S.empty())
      std::memcpy(this->message, S.data(), messageLength);
  }

  bool getStrict() const {
    return strict;
  }

  llvm::StringRef getReplacement() const {
    return llvm::StringRef(replacement, replacementLength);
  }
  unsigned getReplacementLength() const {
    return replacementLength;
  }
  void setReplacement(ASTContext &C, llvm::StringRef S) {
    replacementLength = S.size();
    this->replacement = new (C, 1) char [replacementLength];
    if (!S.empty())
      std::memcpy(this->replacement, S.data(), replacementLength);
  }

  int getPriority() const {
    return priority;
  }

static llvm::StringRef getPrettyPlatformName(llvm::StringRef Platform) {
    return llvm::StringSwitch<llvm::StringRef>(Platform)
             .Case("android", "Android")
             .Case("ios", "iOS")
             .Case("macos", "macOS")
             .Case("tvos", "tvOS")
             .Case("watchos", "watchOS")
             .Case("ios_app_extension", "iOS (App Extension)")
             .Case("macos_app_extension", "macOS (App Extension)")
             .Case("tvos_app_extension", "tvOS (App Extension)")
             .Case("watchos_app_extension", "watchOS (App Extension)")
             .Case("swift", "Swift")
             .Default(llvm::StringRef());
}
static llvm::StringRef getPlatformNameSourceSpelling(llvm::StringRef Platform) {
    return llvm::StringSwitch<llvm::StringRef>(Platform)
             .Case("ios", "iOS")
             .Case("macos", "macOS")
             .Case("tvos", "tvOS")
             .Case("watchos", "watchOS")
             .Case("ios_app_extension", "iOSApplicationExtension")
             .Case("macos_app_extension", "macOSApplicationExtension")
             .Case("tvos_app_extension", "tvOSApplicationExtension")
             .Case("watchos_app_extension", "watchOSApplicationExtension")
             .Default(Platform);
}
static llvm::StringRef canonicalizePlatformName(llvm::StringRef Platform) {
    return llvm::StringSwitch<llvm::StringRef>(Platform)
             .Case("iOS", "ios")
             .Case("macOS", "macos")
             .Case("tvOS", "tvos")
             .Case("watchOS", "watchos")
             .Case("iOSApplicationExtension", "ios_app_extension")
             .Case("macOSApplicationExtension", "macos_app_extension")
             .Case("tvOSApplicationExtension", "tvos_app_extension")
             .Case("watchOSApplicationExtension", "watchos_app_extension")
             .Default(Platform);
} 

  static bool classof(const Attr *A) { return A->getKind() == attr::Availability; }
};

class BPFPreserveAccessIndexAttr : public InheritableAttr {
public:
  static BPFPreserveAccessIndexAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) BPFPreserveAccessIndexAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static BPFPreserveAccessIndexAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) BPFPreserveAccessIndexAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static BPFPreserveAccessIndexAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static BPFPreserveAccessIndexAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  BPFPreserveAccessIndexAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::BPFPreserveAccessIndex, false, false)
  {
  }

  BPFPreserveAccessIndexAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::BPFPreserveAccessIndex; }
};

class BlocksAttr : public InheritableAttr {
public:
  enum BlockType {
    ByRef
  };
private:
  BlockType type;

public:
  static BlocksAttr *CreateImplicit(ASTContext &Ctx, BlockType Type, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) BlocksAttr(Ctx, CommonInfo, Type);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static BlocksAttr *Create(ASTContext &Ctx, BlockType Type, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) BlocksAttr(Ctx, CommonInfo, Type);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static BlocksAttr *CreateImplicit(ASTContext &Ctx, BlockType Type, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Type, I);
  }
  static BlocksAttr *Create(ASTContext &Ctx, BlockType Type, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Type, I);
  }
  BlocksAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , BlockType Type
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Blocks, false, false)
              , type(Type)
  {
  }

  BlocksAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  BlockType getType() const {
    return type;
  }

  static bool ConvertStrToBlockType(StringRef Val, BlockType &Out) {
    Optional<BlockType> R = llvm::StringSwitch<Optional<BlockType>>(Val)
      .Case("byref", BlocksAttr::ByRef)
      .Default(Optional<BlockType>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertBlockTypeToStr(BlockType Val) {
    switch(Val) {
    case BlocksAttr::ByRef: return "byref";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::Blocks; }
};

class C11NoReturnAttr : public InheritableAttr {
public:
  static C11NoReturnAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) C11NoReturnAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static C11NoReturnAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) C11NoReturnAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static C11NoReturnAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static C11NoReturnAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  C11NoReturnAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::C11NoReturn, false, false)
  {
  }

  C11NoReturnAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::C11NoReturn; }
};

class CDeclAttr : public InheritableAttr {
public:
  static CDeclAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CDeclAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CDeclAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CDeclAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CDeclAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CDeclAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CDeclAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CDecl, false, false)
  {
  }

  CDeclAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CDecl; }
};

class CFAuditedTransferAttr : public InheritableAttr {
public:
  static CFAuditedTransferAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFAuditedTransferAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFAuditedTransferAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFAuditedTransferAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFAuditedTransferAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CFAuditedTransferAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CFAuditedTransferAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CFAuditedTransfer, false, false)
  {
  }

  CFAuditedTransferAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CFAuditedTransfer; }
};

class CFConsumedAttr : public InheritableParamAttr {
public:
  static CFConsumedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFConsumedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFConsumedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFConsumedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFConsumedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CFConsumedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CFConsumedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::CFConsumed, false, false)
  {
  }

  CFConsumedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CFConsumed; }
};

class CFGuardAttr : public InheritableAttr {
public:
  enum GuardArg {
    nocf
  };
private:
  GuardArg guard;

public:
  static CFGuardAttr *CreateImplicit(ASTContext &Ctx, GuardArg Guard, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFGuardAttr(Ctx, CommonInfo, Guard);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFGuardAttr *Create(ASTContext &Ctx, GuardArg Guard, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFGuardAttr(Ctx, CommonInfo, Guard);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFGuardAttr *CreateImplicit(ASTContext &Ctx, GuardArg Guard, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Guard, I);
  }
  static CFGuardAttr *Create(ASTContext &Ctx, GuardArg Guard, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Guard, I);
  }
  CFGuardAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , GuardArg Guard
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CFGuard, false, false)
              , guard(Guard)
  {
  }

  CFGuardAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  GuardArg getGuard() const {
    return guard;
  }

  static bool ConvertStrToGuardArg(StringRef Val, GuardArg &Out) {
    Optional<GuardArg> R = llvm::StringSwitch<Optional<GuardArg>>(Val)
      .Case("nocf", CFGuardAttr::nocf)
      .Default(Optional<GuardArg>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertGuardArgToStr(GuardArg Val) {
    switch(Val) {
    case CFGuardAttr::nocf: return "nocf";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::CFGuard; }
};

class CFICanonicalJumpTableAttr : public InheritableAttr {
public:
  static CFICanonicalJumpTableAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFICanonicalJumpTableAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFICanonicalJumpTableAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFICanonicalJumpTableAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFICanonicalJumpTableAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CFICanonicalJumpTableAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CFICanonicalJumpTableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CFICanonicalJumpTable, false, false)
  {
  }

  CFICanonicalJumpTableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CFICanonicalJumpTable; }
};

class CFReturnsNotRetainedAttr : public InheritableAttr {
public:
  static CFReturnsNotRetainedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFReturnsNotRetainedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFReturnsNotRetainedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFReturnsNotRetainedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFReturnsNotRetainedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CFReturnsNotRetainedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CFReturnsNotRetainedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CFReturnsNotRetained, false, false)
  {
  }

  CFReturnsNotRetainedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CFReturnsNotRetained; }
};

class CFReturnsRetainedAttr : public InheritableAttr {
public:
  static CFReturnsRetainedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFReturnsRetainedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFReturnsRetainedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFReturnsRetainedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFReturnsRetainedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CFReturnsRetainedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CFReturnsRetainedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CFReturnsRetained, false, false)
  {
  }

  CFReturnsRetainedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CFReturnsRetained; }
};

class CFUnknownTransferAttr : public InheritableAttr {
public:
  static CFUnknownTransferAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFUnknownTransferAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFUnknownTransferAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CFUnknownTransferAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CFUnknownTransferAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CFUnknownTransferAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CFUnknownTransferAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CFUnknownTransfer, false, false)
  {
  }

  CFUnknownTransferAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CFUnknownTransfer; }
};

class CPUDispatchAttr : public InheritableAttr {
  unsigned cpus_Size;
  IdentifierInfo * *cpus_;

public:
  static CPUDispatchAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * *Cpus, unsigned CpusSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CPUDispatchAttr(Ctx, CommonInfo, Cpus, CpusSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CPUDispatchAttr *Create(ASTContext &Ctx, IdentifierInfo * *Cpus, unsigned CpusSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CPUDispatchAttr(Ctx, CommonInfo, Cpus, CpusSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CPUDispatchAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * *Cpus, unsigned CpusSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Cpus, CpusSize, I);
  }
  static CPUDispatchAttr *Create(ASTContext &Ctx, IdentifierInfo * *Cpus, unsigned CpusSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Cpus, CpusSize, I);
  }
  CPUDispatchAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * *Cpus, unsigned CpusSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CPUDispatch, false, false)
              , cpus_Size(CpusSize), cpus_(new (Ctx, 16) IdentifierInfo *[cpus_Size])
  {
    std::copy(Cpus, Cpus + cpus_Size, cpus_);
  }

  CPUDispatchAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CPUDispatch, false, false)
              , cpus_Size(0), cpus_(nullptr)
  {
  }

  CPUDispatchAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef IdentifierInfo ** cpus_iterator;
  cpus_iterator cpus_begin() const { return cpus_; }
  cpus_iterator cpus_end() const { return cpus_ + cpus_Size; }
  unsigned cpus_size() const { return cpus_Size; }
  llvm::iterator_range<cpus_iterator> cpus() const { return llvm::make_range(cpus_begin(), cpus_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::CPUDispatch; }
};

class CPUSpecificAttr : public InheritableAttr {
  unsigned cpus_Size;
  IdentifierInfo * *cpus_;

public:
  static CPUSpecificAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * *Cpus, unsigned CpusSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CPUSpecificAttr(Ctx, CommonInfo, Cpus, CpusSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CPUSpecificAttr *Create(ASTContext &Ctx, IdentifierInfo * *Cpus, unsigned CpusSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CPUSpecificAttr(Ctx, CommonInfo, Cpus, CpusSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CPUSpecificAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * *Cpus, unsigned CpusSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Cpus, CpusSize, I);
  }
  static CPUSpecificAttr *Create(ASTContext &Ctx, IdentifierInfo * *Cpus, unsigned CpusSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Cpus, CpusSize, I);
  }
  CPUSpecificAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * *Cpus, unsigned CpusSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CPUSpecific, false, false)
              , cpus_Size(CpusSize), cpus_(new (Ctx, 16) IdentifierInfo *[cpus_Size])
  {
    std::copy(Cpus, Cpus + cpus_Size, cpus_);
  }

  CPUSpecificAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CPUSpecific, false, false)
              , cpus_Size(0), cpus_(nullptr)
  {
  }

  CPUSpecificAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef IdentifierInfo ** cpus_iterator;
  cpus_iterator cpus_begin() const { return cpus_; }
  cpus_iterator cpus_end() const { return cpus_ + cpus_Size; }
  unsigned cpus_size() const { return cpus_Size; }
  llvm::iterator_range<cpus_iterator> cpus() const { return llvm::make_range(cpus_begin(), cpus_end()); }



    IdentifierInfo *getCPUName(unsigned Index) const {
      return *(cpus_begin() + Index);
    }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::CPUSpecific; }
};

class CUDAConstantAttr : public InheritableAttr {
public:
  static CUDAConstantAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDAConstantAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDAConstantAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDAConstantAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDAConstantAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CUDAConstantAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CUDAConstantAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CUDAConstant, false, false)
  {
  }

  CUDAConstantAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CUDAConstant; }
};

class CUDADeviceAttr : public InheritableAttr {
public:
  static CUDADeviceAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDADeviceAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDADeviceAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDADeviceAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDADeviceAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CUDADeviceAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CUDADeviceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CUDADevice, false, false)
  {
  }

  CUDADeviceAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CUDADevice; }
};

class CUDAGlobalAttr : public InheritableAttr {
public:
  static CUDAGlobalAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDAGlobalAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDAGlobalAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDAGlobalAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDAGlobalAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CUDAGlobalAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CUDAGlobalAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CUDAGlobal, false, false)
  {
  }

  CUDAGlobalAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CUDAGlobal; }
};

class CUDAHostAttr : public InheritableAttr {
public:
  static CUDAHostAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDAHostAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDAHostAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDAHostAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDAHostAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CUDAHostAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CUDAHostAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CUDAHost, false, false)
  {
  }

  CUDAHostAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CUDAHost; }
};

class CUDAInvalidTargetAttr : public InheritableAttr {
public:
  static CUDAInvalidTargetAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDAInvalidTargetAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDAInvalidTargetAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDAInvalidTargetAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDAInvalidTargetAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CUDAInvalidTargetAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CUDAInvalidTargetAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CUDAInvalidTarget, false, false)
  {
  }

  CUDAInvalidTargetAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CUDAInvalidTarget; }
};

class CUDALaunchBoundsAttr : public InheritableAttr {
Expr * maxThreads;

Expr * minBlocks;

public:
  static CUDALaunchBoundsAttr *CreateImplicit(ASTContext &Ctx, Expr * MaxThreads, Expr * MinBlocks, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDALaunchBoundsAttr(Ctx, CommonInfo, MaxThreads, MinBlocks);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDALaunchBoundsAttr *Create(ASTContext &Ctx, Expr * MaxThreads, Expr * MinBlocks, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDALaunchBoundsAttr(Ctx, CommonInfo, MaxThreads, MinBlocks);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDALaunchBoundsAttr *CreateImplicit(ASTContext &Ctx, Expr * MaxThreads, Expr * MinBlocks, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, MaxThreads, MinBlocks, I);
  }
  static CUDALaunchBoundsAttr *Create(ASTContext &Ctx, Expr * MaxThreads, Expr * MinBlocks, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, MaxThreads, MinBlocks, I);
  }
  CUDALaunchBoundsAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * MaxThreads
              , Expr * MinBlocks
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CUDALaunchBounds, false, false)
              , maxThreads(MaxThreads)
              , minBlocks(MinBlocks)
  {
  }

  CUDALaunchBoundsAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * MaxThreads
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CUDALaunchBounds, false, false)
              , maxThreads(MaxThreads)
              , minBlocks()
  {
  }

  CUDALaunchBoundsAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getMaxThreads() const {
    return maxThreads;
  }

  Expr * getMinBlocks() const {
    return minBlocks;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::CUDALaunchBounds; }
};

class CUDASharedAttr : public InheritableAttr {
public:
  static CUDASharedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDASharedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDASharedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CUDASharedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CUDASharedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CUDASharedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CUDASharedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CUDAShared, false, false)
  {
  }

  CUDASharedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CUDAShared; }
};

class CXX11NoReturnAttr : public InheritableAttr {
public:
  static CXX11NoReturnAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CXX11NoReturnAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CXX11NoReturnAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CXX11NoReturnAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CXX11NoReturnAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CXX11NoReturnAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CXX11NoReturnAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CXX11NoReturn, false, false)
  {
  }

  CXX11NoReturnAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CXX11NoReturn; }
};

class CallableWhenAttr : public InheritableAttr {
public:
  enum ConsumedState {
    Unknown,
    Consumed,
    Unconsumed
  };
private:
  unsigned callableStates_Size;
  ConsumedState *callableStates_;

public:
  static CallableWhenAttr *CreateImplicit(ASTContext &Ctx, ConsumedState *CallableStates, unsigned CallableStatesSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CallableWhenAttr(Ctx, CommonInfo, CallableStates, CallableStatesSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CallableWhenAttr *Create(ASTContext &Ctx, ConsumedState *CallableStates, unsigned CallableStatesSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CallableWhenAttr(Ctx, CommonInfo, CallableStates, CallableStatesSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CallableWhenAttr *CreateImplicit(ASTContext &Ctx, ConsumedState *CallableStates, unsigned CallableStatesSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, CallableStates, CallableStatesSize, I);
  }
  static CallableWhenAttr *Create(ASTContext &Ctx, ConsumedState *CallableStates, unsigned CallableStatesSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, CallableStates, CallableStatesSize, I);
  }
  CallableWhenAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ConsumedState *CallableStates, unsigned CallableStatesSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CallableWhen, false, false)
              , callableStates_Size(CallableStatesSize), callableStates_(new (Ctx, 16) ConsumedState[callableStates_Size])
  {
    std::copy(CallableStates, CallableStates + callableStates_Size, callableStates_);
  }

  CallableWhenAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CallableWhen, false, false)
              , callableStates_Size(0), callableStates_(nullptr)
  {
  }

  CallableWhenAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef ConsumedState* callableStates_iterator;
  callableStates_iterator callableStates_begin() const { return callableStates_; }
  callableStates_iterator callableStates_end() const { return callableStates_ + callableStates_Size; }
  unsigned callableStates_size() const { return callableStates_Size; }
  llvm::iterator_range<callableStates_iterator> callableStates() const { return llvm::make_range(callableStates_begin(), callableStates_end()); }


  static bool ConvertStrToConsumedState(StringRef Val, ConsumedState &Out) {
    Optional<ConsumedState> R = llvm::StringSwitch<Optional<ConsumedState>>(Val)
      .Case("unknown", CallableWhenAttr::Unknown)
      .Case("consumed", CallableWhenAttr::Consumed)
      .Case("unconsumed", CallableWhenAttr::Unconsumed)
      .Default(Optional<ConsumedState>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertConsumedStateToStr(ConsumedState Val) {
    switch(Val) {
    case CallableWhenAttr::Unknown: return "unknown";
    case CallableWhenAttr::Consumed: return "consumed";
    case CallableWhenAttr::Unconsumed: return "unconsumed";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::CallableWhen; }
};

class CallbackAttr : public InheritableAttr {
  unsigned encoding_Size;
  int *encoding_;

public:
  static CallbackAttr *CreateImplicit(ASTContext &Ctx, int *Encoding, unsigned EncodingSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CallbackAttr(Ctx, CommonInfo, Encoding, EncodingSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CallbackAttr *Create(ASTContext &Ctx, int *Encoding, unsigned EncodingSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CallbackAttr(Ctx, CommonInfo, Encoding, EncodingSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CallbackAttr *CreateImplicit(ASTContext &Ctx, int *Encoding, unsigned EncodingSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Encoding, EncodingSize, I);
  }
  static CallbackAttr *Create(ASTContext &Ctx, int *Encoding, unsigned EncodingSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Encoding, EncodingSize, I);
  }
  CallbackAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , int *Encoding, unsigned EncodingSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Callback, false, false)
              , encoding_Size(EncodingSize), encoding_(new (Ctx, 16) int[encoding_Size])
  {
    std::copy(Encoding, Encoding + encoding_Size, encoding_);
  }

  CallbackAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Callback, false, false)
              , encoding_Size(0), encoding_(nullptr)
  {
  }

  CallbackAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef int* encoding_iterator;
  encoding_iterator encoding_begin() const { return encoding_; }
  encoding_iterator encoding_end() const { return encoding_ + encoding_Size; }
  unsigned encoding_size() const { return encoding_Size; }
  llvm::iterator_range<encoding_iterator> encoding() const { return llvm::make_range(encoding_begin(), encoding_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::Callback; }
};

class CapabilityAttr : public InheritableAttr {
unsigned nameLength;
char *name;

public:
  enum Spelling {
    GNU_capability = 0,
    CXX11_clang_capability = 1,
    GNU_shared_capability = 2,
    CXX11_clang_shared_capability = 3,
  SpellingNotCalculated = 15

  };

  static CapabilityAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CapabilityAttr(Ctx, CommonInfo, Name);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CapabilityAttr *Create(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CapabilityAttr(Ctx, CommonInfo, Name);
    return A;
  }

  static CapabilityAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax, CapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Name, I);
  }
  static CapabilityAttr *Create(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax, CapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Name, I);
  }
  CapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Name
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Capability, false, false)
              , nameLength(Name.size()),name(new (Ctx, 1) char[nameLength])
  {
      if (!Name.empty())
        std::memcpy(name, Name.data(), nameLength);
  }

  CapabilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_capability;
    case 1: return CXX11_clang_capability;
    case 2: return GNU_shared_capability;
    case 3: return CXX11_clang_shared_capability;
  }
  }
  bool isShared() const { return getAttributeSpellingListIndex() == 2 ||
    getAttributeSpellingListIndex() == 3; }
  llvm::StringRef getName() const {
    return llvm::StringRef(name, nameLength);
  }
  unsigned getNameLength() const {
    return nameLength;
  }
  void setName(ASTContext &C, llvm::StringRef S) {
    nameLength = S.size();
    this->name = new (C, 1) char [nameLength];
    if (!S.empty())
      std::memcpy(this->name, S.data(), nameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Capability; }
};

class CapturedRecordAttr : public InheritableAttr {
public:
  static CapturedRecordAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CapturedRecordAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CapturedRecordAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CapturedRecordAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CapturedRecordAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CapturedRecordAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CapturedRecordAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CapturedRecord, false, false)
  {
  }

  CapturedRecordAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CapturedRecord; }
};

class CarriesDependencyAttr : public InheritableParamAttr {
public:
  static CarriesDependencyAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CarriesDependencyAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CarriesDependencyAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CarriesDependencyAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CarriesDependencyAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CarriesDependencyAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CarriesDependencyAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::CarriesDependency, false, false)
  {
  }

  CarriesDependencyAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::CarriesDependency; }
};

class CleanupAttr : public InheritableAttr {
FunctionDecl * functionDecl;

public:
  static CleanupAttr *CreateImplicit(ASTContext &Ctx, FunctionDecl * FunctionDecl, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CleanupAttr(Ctx, CommonInfo, FunctionDecl);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CleanupAttr *Create(ASTContext &Ctx, FunctionDecl * FunctionDecl, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CleanupAttr(Ctx, CommonInfo, FunctionDecl);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CleanupAttr *CreateImplicit(ASTContext &Ctx, FunctionDecl * FunctionDecl, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, FunctionDecl, I);
  }
  static CleanupAttr *Create(ASTContext &Ctx, FunctionDecl * FunctionDecl, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, FunctionDecl, I);
  }
  CleanupAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , FunctionDecl * FunctionDecl
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Cleanup, false, false)
              , functionDecl(FunctionDecl)
  {
  }

  CleanupAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  FunctionDecl * getFunctionDecl() const {
    return functionDecl;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Cleanup; }
};

class CodeSegAttr : public InheritableAttr {
unsigned nameLength;
char *name;

public:
  static CodeSegAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CodeSegAttr(Ctx, CommonInfo, Name);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CodeSegAttr *Create(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CodeSegAttr(Ctx, CommonInfo, Name);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CodeSegAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Name, I);
  }
  static CodeSegAttr *Create(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Name, I);
  }
  CodeSegAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Name
             )
    : InheritableAttr(Ctx, CommonInfo, attr::CodeSeg, false, false)
              , nameLength(Name.size()),name(new (Ctx, 1) char[nameLength])
  {
      if (!Name.empty())
        std::memcpy(name, Name.data(), nameLength);
  }

  CodeSegAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getName() const {
    return llvm::StringRef(name, nameLength);
  }
  unsigned getNameLength() const {
    return nameLength;
  }
  void setName(ASTContext &C, llvm::StringRef S) {
    nameLength = S.size();
    this->name = new (C, 1) char [nameLength];
    if (!S.empty())
      std::memcpy(this->name, S.data(), nameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::CodeSeg; }
};

class ColdAttr : public InheritableAttr {
public:
  static ColdAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ColdAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ColdAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ColdAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ColdAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ColdAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ColdAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Cold, false, false)
  {
  }

  ColdAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Cold; }
};

class CommonAttr : public InheritableAttr {
public:
  static CommonAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CommonAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CommonAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) CommonAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static CommonAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static CommonAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  CommonAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Common, false, false)
  {
  }

  CommonAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Common; }
};

class ConstAttr : public InheritableAttr {
public:
  static ConstAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConstAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConstAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConstAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConstAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ConstAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ConstAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Const, false, false)
  {
  }

  ConstAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Const; }
};

class ConstInitAttr : public InheritableAttr {
public:
  enum Spelling {
    Keyword_constinit = 0,
    GNU_require_constant_initialization = 1,
    CXX11_clang_require_constant_initialization = 2,
  SpellingNotCalculated = 15

  };

  static ConstInitAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConstInitAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConstInitAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConstInitAttr(Ctx, CommonInfo);
    return A;
  }

  static ConstInitAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, ConstInitAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static ConstInitAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, ConstInitAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  ConstInitAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ConstInit, false, false)
  {
  }

  ConstInitAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Keyword_constinit;
    case 1: return GNU_require_constant_initialization;
    case 2: return CXX11_clang_require_constant_initialization;
  }
  }
  bool isConstinit() const { return getAttributeSpellingListIndex() == 0; }


  static bool classof(const Attr *A) { return A->getKind() == attr::ConstInit; }
};

class ConstructorAttr : public InheritableAttr {
int priority;

public:
  static ConstructorAttr *CreateImplicit(ASTContext &Ctx, int Priority, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConstructorAttr(Ctx, CommonInfo, Priority);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConstructorAttr *Create(ASTContext &Ctx, int Priority, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConstructorAttr(Ctx, CommonInfo, Priority);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConstructorAttr *CreateImplicit(ASTContext &Ctx, int Priority, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Priority, I);
  }
  static ConstructorAttr *Create(ASTContext &Ctx, int Priority, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Priority, I);
  }
  ConstructorAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , int Priority
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Constructor, false, false)
              , priority(Priority)
  {
  }

  ConstructorAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Constructor, false, false)
              , priority()
  {
  }

  ConstructorAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  int getPriority() const {
    return priority;
  }

  static const int DefaultPriority = 65535;



  static bool classof(const Attr *A) { return A->getKind() == attr::Constructor; }
};

class ConsumableAttr : public InheritableAttr {
public:
  enum ConsumedState {
    Unknown,
    Consumed,
    Unconsumed
  };
private:
  ConsumedState defaultState;

public:
  static ConsumableAttr *CreateImplicit(ASTContext &Ctx, ConsumedState DefaultState, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConsumableAttr(Ctx, CommonInfo, DefaultState);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConsumableAttr *Create(ASTContext &Ctx, ConsumedState DefaultState, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConsumableAttr(Ctx, CommonInfo, DefaultState);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConsumableAttr *CreateImplicit(ASTContext &Ctx, ConsumedState DefaultState, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, DefaultState, I);
  }
  static ConsumableAttr *Create(ASTContext &Ctx, ConsumedState DefaultState, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, DefaultState, I);
  }
  ConsumableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ConsumedState DefaultState
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Consumable, false, false)
              , defaultState(DefaultState)
  {
  }

  ConsumableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  ConsumedState getDefaultState() const {
    return defaultState;
  }

  static bool ConvertStrToConsumedState(StringRef Val, ConsumedState &Out) {
    Optional<ConsumedState> R = llvm::StringSwitch<Optional<ConsumedState>>(Val)
      .Case("unknown", ConsumableAttr::Unknown)
      .Case("consumed", ConsumableAttr::Consumed)
      .Case("unconsumed", ConsumableAttr::Unconsumed)
      .Default(Optional<ConsumedState>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertConsumedStateToStr(ConsumedState Val) {
    switch(Val) {
    case ConsumableAttr::Unknown: return "unknown";
    case ConsumableAttr::Consumed: return "consumed";
    case ConsumableAttr::Unconsumed: return "unconsumed";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::Consumable; }
};

class ConsumableAutoCastAttr : public InheritableAttr {
public:
  static ConsumableAutoCastAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConsumableAutoCastAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConsumableAutoCastAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConsumableAutoCastAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConsumableAutoCastAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ConsumableAutoCastAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ConsumableAutoCastAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ConsumableAutoCast, false, false)
  {
  }

  ConsumableAutoCastAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ConsumableAutoCast; }
};

class ConsumableSetOnReadAttr : public InheritableAttr {
public:
  static ConsumableSetOnReadAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConsumableSetOnReadAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConsumableSetOnReadAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConsumableSetOnReadAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConsumableSetOnReadAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ConsumableSetOnReadAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ConsumableSetOnReadAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ConsumableSetOnRead, false, false)
  {
  }

  ConsumableSetOnReadAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ConsumableSetOnRead; }
};

class ConvergentAttr : public InheritableAttr {
public:
  static ConvergentAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConvergentAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConvergentAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ConvergentAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ConvergentAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ConvergentAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ConvergentAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Convergent, false, false)
  {
  }

  ConvergentAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Convergent; }
};

class DLLExportAttr : public InheritableAttr {
public:
  static DLLExportAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DLLExportAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DLLExportAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DLLExportAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DLLExportAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static DLLExportAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  DLLExportAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::DLLExport, false, false)
  {
  }

  DLLExportAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::DLLExport; }
};

class DLLExportStaticLocalAttr : public InheritableAttr {
public:
  static DLLExportStaticLocalAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DLLExportStaticLocalAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DLLExportStaticLocalAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DLLExportStaticLocalAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DLLExportStaticLocalAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static DLLExportStaticLocalAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  DLLExportStaticLocalAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::DLLExportStaticLocal, false, false)
  {
  }

  DLLExportStaticLocalAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::DLLExportStaticLocal; }
};

class DLLImportAttr : public InheritableAttr {
public:
  static DLLImportAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DLLImportAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DLLImportAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DLLImportAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DLLImportAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static DLLImportAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  DLLImportAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::DLLImport, false, false)
  {
  }

  DLLImportAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;

private:
  bool PropagatedToBaseTemplate = false;

public:
  void setPropagatedToBaseTemplate() { PropagatedToBaseTemplate = true; }
  bool wasPropagatedToBaseTemplate() { return PropagatedToBaseTemplate; }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::DLLImport; }
};

class DLLImportStaticLocalAttr : public InheritableAttr {
public:
  static DLLImportStaticLocalAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DLLImportStaticLocalAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DLLImportStaticLocalAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DLLImportStaticLocalAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DLLImportStaticLocalAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static DLLImportStaticLocalAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  DLLImportStaticLocalAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::DLLImportStaticLocal, false, false)
  {
  }

  DLLImportStaticLocalAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::DLLImportStaticLocal; }
};

class DeprecatedAttr : public InheritableAttr {
unsigned messageLength;
char *message;

unsigned replacementLength;
char *replacement;

public:
  static DeprecatedAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Message, llvm::StringRef Replacement, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DeprecatedAttr(Ctx, CommonInfo, Message, Replacement);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DeprecatedAttr *Create(ASTContext &Ctx, llvm::StringRef Message, llvm::StringRef Replacement, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DeprecatedAttr(Ctx, CommonInfo, Message, Replacement);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DeprecatedAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Message, llvm::StringRef Replacement, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Message, Replacement, I);
  }
  static DeprecatedAttr *Create(ASTContext &Ctx, llvm::StringRef Message, llvm::StringRef Replacement, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Message, Replacement, I);
  }
  DeprecatedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Message
              , llvm::StringRef Replacement
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Deprecated, false, false)
              , messageLength(Message.size()),message(new (Ctx, 1) char[messageLength])
              , replacementLength(Replacement.size()),replacement(new (Ctx, 1) char[replacementLength])
  {
      if (!Message.empty())
        std::memcpy(message, Message.data(), messageLength);
      if (!Replacement.empty())
        std::memcpy(replacement, Replacement.data(), replacementLength);
  }

  DeprecatedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Deprecated, false, false)
              , messageLength(0),message(nullptr)
              , replacementLength(0),replacement(nullptr)
  {
  }

  DeprecatedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getMessage() const {
    return llvm::StringRef(message, messageLength);
  }
  unsigned getMessageLength() const {
    return messageLength;
  }
  void setMessage(ASTContext &C, llvm::StringRef S) {
    messageLength = S.size();
    this->message = new (C, 1) char [messageLength];
    if (!S.empty())
      std::memcpy(this->message, S.data(), messageLength);
  }

  llvm::StringRef getReplacement() const {
    return llvm::StringRef(replacement, replacementLength);
  }
  unsigned getReplacementLength() const {
    return replacementLength;
  }
  void setReplacement(ASTContext &C, llvm::StringRef S) {
    replacementLength = S.size();
    this->replacement = new (C, 1) char [replacementLength];
    if (!S.empty())
      std::memcpy(this->replacement, S.data(), replacementLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Deprecated; }
};

class DestructorAttr : public InheritableAttr {
int priority;

public:
  static DestructorAttr *CreateImplicit(ASTContext &Ctx, int Priority, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DestructorAttr(Ctx, CommonInfo, Priority);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DestructorAttr *Create(ASTContext &Ctx, int Priority, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DestructorAttr(Ctx, CommonInfo, Priority);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DestructorAttr *CreateImplicit(ASTContext &Ctx, int Priority, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Priority, I);
  }
  static DestructorAttr *Create(ASTContext &Ctx, int Priority, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Priority, I);
  }
  DestructorAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , int Priority
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Destructor, false, false)
              , priority(Priority)
  {
  }

  DestructorAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Destructor, false, false)
              , priority()
  {
  }

  DestructorAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  int getPriority() const {
    return priority;
  }

  static const int DefaultPriority = 65535;



  static bool classof(const Attr *A) { return A->getKind() == attr::Destructor; }
};

class DiagnoseIfAttr : public InheritableAttr {
Expr * cond;

unsigned messageLength;
char *message;

public:
  enum DiagnosticType {
    DT_Error,
    DT_Warning
  };
private:
  DiagnosticType diagnosticType;

bool argDependent;

NamedDecl * parent;

public:
  static DiagnoseIfAttr *CreateImplicit(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, DiagnosticType DiagnosticType, bool ArgDependent, NamedDecl * Parent, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DiagnoseIfAttr(Ctx, CommonInfo, Cond, Message, DiagnosticType, ArgDependent, Parent);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DiagnoseIfAttr *Create(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, DiagnosticType DiagnosticType, bool ArgDependent, NamedDecl * Parent, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DiagnoseIfAttr(Ctx, CommonInfo, Cond, Message, DiagnosticType, ArgDependent, Parent);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DiagnoseIfAttr *CreateImplicit(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, DiagnosticType DiagnosticType, bool ArgDependent, NamedDecl * Parent, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Cond, Message, DiagnosticType, ArgDependent, Parent, I);
  }
  static DiagnoseIfAttr *Create(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, DiagnosticType DiagnosticType, bool ArgDependent, NamedDecl * Parent, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Cond, Message, DiagnosticType, ArgDependent, Parent, I);
  }
  static DiagnoseIfAttr *CreateImplicit(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, DiagnosticType DiagnosticType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DiagnoseIfAttr(Ctx, CommonInfo, Cond, Message, DiagnosticType);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DiagnoseIfAttr *Create(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, DiagnosticType DiagnosticType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DiagnoseIfAttr(Ctx, CommonInfo, Cond, Message, DiagnosticType);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DiagnoseIfAttr *CreateImplicit(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, DiagnosticType DiagnosticType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Cond, Message, DiagnosticType, I);
  }
  static DiagnoseIfAttr *Create(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, DiagnosticType DiagnosticType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Cond, Message, DiagnosticType, I);
  }
  DiagnoseIfAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Cond
              , llvm::StringRef Message
              , DiagnosticType DiagnosticType
              , bool ArgDependent
              , NamedDecl * Parent
             )
    : InheritableAttr(Ctx, CommonInfo, attr::DiagnoseIf, true, true)
              , cond(Cond)
              , messageLength(Message.size()),message(new (Ctx, 1) char[messageLength])
              , diagnosticType(DiagnosticType)
              , argDependent(ArgDependent)
              , parent(Parent)
  {
      if (!Message.empty())
        std::memcpy(message, Message.data(), messageLength);
  }

  DiagnoseIfAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Cond
              , llvm::StringRef Message
              , DiagnosticType DiagnosticType
             )
    : InheritableAttr(Ctx, CommonInfo, attr::DiagnoseIf, true, true)
              , cond(Cond)
              , messageLength(Message.size()),message(new (Ctx, 1) char[messageLength])
              , diagnosticType(DiagnosticType)
              , argDependent()
              , parent()
  {
      if (!Message.empty())
        std::memcpy(message, Message.data(), messageLength);
  }

  DiagnoseIfAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getCond() const {
    return cond;
  }

  llvm::StringRef getMessage() const {
    return llvm::StringRef(message, messageLength);
  }
  unsigned getMessageLength() const {
    return messageLength;
  }
  void setMessage(ASTContext &C, llvm::StringRef S) {
    messageLength = S.size();
    this->message = new (C, 1) char [messageLength];
    if (!S.empty())
      std::memcpy(this->message, S.data(), messageLength);
  }

  DiagnosticType getDiagnosticType() const {
    return diagnosticType;
  }

  static bool ConvertStrToDiagnosticType(StringRef Val, DiagnosticType &Out) {
    Optional<DiagnosticType> R = llvm::StringSwitch<Optional<DiagnosticType>>(Val)
      .Case("error", DiagnoseIfAttr::DT_Error)
      .Case("warning", DiagnoseIfAttr::DT_Warning)
      .Default(Optional<DiagnosticType>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertDiagnosticTypeToStr(DiagnosticType Val) {
    switch(Val) {
    case DiagnoseIfAttr::DT_Error: return "error";
    case DiagnoseIfAttr::DT_Warning: return "warning";
    }
    llvm_unreachable("No enumerator with that value");
  }
  bool getArgDependent() const {
    return argDependent;
  }

  NamedDecl * getParent() const {
    return parent;
  }


    bool isError() const { return diagnosticType == DT_Error; }
    bool isWarning() const { return diagnosticType == DT_Warning; }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::DiagnoseIf; }
};

class DisableTailCallsAttr : public InheritableAttr {
public:
  static DisableTailCallsAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DisableTailCallsAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DisableTailCallsAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) DisableTailCallsAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static DisableTailCallsAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static DisableTailCallsAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  DisableTailCallsAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::DisableTailCalls, false, false)
  {
  }

  DisableTailCallsAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::DisableTailCalls; }
};

class EmptyBasesAttr : public InheritableAttr {
public:
  static EmptyBasesAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) EmptyBasesAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static EmptyBasesAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) EmptyBasesAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static EmptyBasesAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static EmptyBasesAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  EmptyBasesAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::EmptyBases, false, false)
  {
  }

  EmptyBasesAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::EmptyBases; }
};

class EnableIfAttr : public InheritableAttr {
Expr * cond;

unsigned messageLength;
char *message;

public:
  static EnableIfAttr *CreateImplicit(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) EnableIfAttr(Ctx, CommonInfo, Cond, Message);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static EnableIfAttr *Create(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) EnableIfAttr(Ctx, CommonInfo, Cond, Message);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static EnableIfAttr *CreateImplicit(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Cond, Message, I);
  }
  static EnableIfAttr *Create(ASTContext &Ctx, Expr * Cond, llvm::StringRef Message, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Cond, Message, I);
  }
  EnableIfAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Cond
              , llvm::StringRef Message
             )
    : InheritableAttr(Ctx, CommonInfo, attr::EnableIf, false, false)
              , cond(Cond)
              , messageLength(Message.size()),message(new (Ctx, 1) char[messageLength])
  {
      if (!Message.empty())
        std::memcpy(message, Message.data(), messageLength);
  }

  EnableIfAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getCond() const {
    return cond;
  }

  llvm::StringRef getMessage() const {
    return llvm::StringRef(message, messageLength);
  }
  unsigned getMessageLength() const {
    return messageLength;
  }
  void setMessage(ASTContext &C, llvm::StringRef S) {
    messageLength = S.size();
    this->message = new (C, 1) char [messageLength];
    if (!S.empty())
      std::memcpy(this->message, S.data(), messageLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::EnableIf; }
};

class EnumExtensibilityAttr : public InheritableAttr {
public:
  enum Kind {
    Closed,
    Open
  };
private:
  Kind extensibility;

public:
  static EnumExtensibilityAttr *CreateImplicit(ASTContext &Ctx, Kind Extensibility, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) EnumExtensibilityAttr(Ctx, CommonInfo, Extensibility);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static EnumExtensibilityAttr *Create(ASTContext &Ctx, Kind Extensibility, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) EnumExtensibilityAttr(Ctx, CommonInfo, Extensibility);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static EnumExtensibilityAttr *CreateImplicit(ASTContext &Ctx, Kind Extensibility, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Extensibility, I);
  }
  static EnumExtensibilityAttr *Create(ASTContext &Ctx, Kind Extensibility, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Extensibility, I);
  }
  EnumExtensibilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Kind Extensibility
             )
    : InheritableAttr(Ctx, CommonInfo, attr::EnumExtensibility, false, false)
              , extensibility(Extensibility)
  {
  }

  EnumExtensibilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Kind getExtensibility() const {
    return extensibility;
  }

  static bool ConvertStrToKind(StringRef Val, Kind &Out) {
    Optional<Kind> R = llvm::StringSwitch<Optional<Kind>>(Val)
      .Case("closed", EnumExtensibilityAttr::Closed)
      .Case("open", EnumExtensibilityAttr::Open)
      .Default(Optional<Kind>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertKindToStr(Kind Val) {
    switch(Val) {
    case EnumExtensibilityAttr::Closed: return "closed";
    case EnumExtensibilityAttr::Open: return "open";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::EnumExtensibility; }
};

class ExcludeFromExplicitInstantiationAttr : public InheritableAttr {
public:
  static ExcludeFromExplicitInstantiationAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ExcludeFromExplicitInstantiationAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ExcludeFromExplicitInstantiationAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ExcludeFromExplicitInstantiationAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ExcludeFromExplicitInstantiationAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ExcludeFromExplicitInstantiationAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ExcludeFromExplicitInstantiationAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ExcludeFromExplicitInstantiation, false, false)
  {
  }

  ExcludeFromExplicitInstantiationAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ExcludeFromExplicitInstantiation; }
};

class ExclusiveTrylockFunctionAttr : public InheritableAttr {
Expr * successValue;

  unsigned args_Size;
  Expr * *args_;

public:
  static ExclusiveTrylockFunctionAttr *CreateImplicit(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ExclusiveTrylockFunctionAttr(Ctx, CommonInfo, SuccessValue, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ExclusiveTrylockFunctionAttr *Create(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ExclusiveTrylockFunctionAttr(Ctx, CommonInfo, SuccessValue, Args, ArgsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ExclusiveTrylockFunctionAttr *CreateImplicit(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, SuccessValue, Args, ArgsSize, I);
  }
  static ExclusiveTrylockFunctionAttr *Create(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, SuccessValue, Args, ArgsSize, I);
  }
  ExclusiveTrylockFunctionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * SuccessValue
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ExclusiveTrylockFunction, true, true)
              , successValue(SuccessValue)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  ExclusiveTrylockFunctionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * SuccessValue
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ExclusiveTrylockFunction, true, true)
              , successValue(SuccessValue)
              , args_Size(0), args_(nullptr)
  {
  }

  ExclusiveTrylockFunctionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getSuccessValue() const {
    return successValue;
  }

  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::ExclusiveTrylockFunction; }
};

class ExternalSourceSymbolAttr : public InheritableAttr {
unsigned languageLength;
char *language;

unsigned definedInLength;
char *definedIn;

bool generatedDeclaration;

public:
  static ExternalSourceSymbolAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Language, llvm::StringRef DefinedIn, bool GeneratedDeclaration, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ExternalSourceSymbolAttr(Ctx, CommonInfo, Language, DefinedIn, GeneratedDeclaration);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ExternalSourceSymbolAttr *Create(ASTContext &Ctx, llvm::StringRef Language, llvm::StringRef DefinedIn, bool GeneratedDeclaration, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ExternalSourceSymbolAttr(Ctx, CommonInfo, Language, DefinedIn, GeneratedDeclaration);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ExternalSourceSymbolAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Language, llvm::StringRef DefinedIn, bool GeneratedDeclaration, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Language, DefinedIn, GeneratedDeclaration, I);
  }
  static ExternalSourceSymbolAttr *Create(ASTContext &Ctx, llvm::StringRef Language, llvm::StringRef DefinedIn, bool GeneratedDeclaration, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Language, DefinedIn, GeneratedDeclaration, I);
  }
  ExternalSourceSymbolAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Language
              , llvm::StringRef DefinedIn
              , bool GeneratedDeclaration
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ExternalSourceSymbol, false, false)
              , languageLength(Language.size()),language(new (Ctx, 1) char[languageLength])
              , definedInLength(DefinedIn.size()),definedIn(new (Ctx, 1) char[definedInLength])
              , generatedDeclaration(GeneratedDeclaration)
  {
      if (!Language.empty())
        std::memcpy(language, Language.data(), languageLength);
      if (!DefinedIn.empty())
        std::memcpy(definedIn, DefinedIn.data(), definedInLength);
  }

  ExternalSourceSymbolAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ExternalSourceSymbol, false, false)
              , languageLength(0),language(nullptr)
              , definedInLength(0),definedIn(nullptr)
              , generatedDeclaration()
  {
  }

  ExternalSourceSymbolAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getLanguage() const {
    return llvm::StringRef(language, languageLength);
  }
  unsigned getLanguageLength() const {
    return languageLength;
  }
  void setLanguage(ASTContext &C, llvm::StringRef S) {
    languageLength = S.size();
    this->language = new (C, 1) char [languageLength];
    if (!S.empty())
      std::memcpy(this->language, S.data(), languageLength);
  }

  llvm::StringRef getDefinedIn() const {
    return llvm::StringRef(definedIn, definedInLength);
  }
  unsigned getDefinedInLength() const {
    return definedInLength;
  }
  void setDefinedIn(ASTContext &C, llvm::StringRef S) {
    definedInLength = S.size();
    this->definedIn = new (C, 1) char [definedInLength];
    if (!S.empty())
      std::memcpy(this->definedIn, S.data(), definedInLength);
  }

  bool getGeneratedDeclaration() const {
    return generatedDeclaration;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ExternalSourceSymbol; }
};

class FallThroughAttr : public StmtAttr {
public:
  static FallThroughAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FallThroughAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FallThroughAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FallThroughAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FallThroughAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static FallThroughAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  FallThroughAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : StmtAttr(Ctx, CommonInfo, attr::FallThrough, false)
  {
  }

  FallThroughAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::FallThrough; }
};

class FastCallAttr : public InheritableAttr {
public:
  static FastCallAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FastCallAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FastCallAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FastCallAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FastCallAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static FastCallAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  FastCallAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::FastCall, false, false)
  {
  }

  FastCallAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::FastCall; }
};

class FinalAttr : public InheritableAttr {
public:
  enum Spelling {
    Keyword_final = 0,
    Keyword_sealed = 1,
  SpellingNotCalculated = 15

  };

  static FinalAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FinalAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FinalAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FinalAttr(Ctx, CommonInfo);
    return A;
  }

  static FinalAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, FinalAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static FinalAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, FinalAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  FinalAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Final, false, false)
  {
  }

  FinalAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Keyword_final;
    case 1: return Keyword_sealed;
  }
  }
  bool isSpelledAsSealed() const { return getAttributeSpellingListIndex() == 1; }


  static bool classof(const Attr *A) { return A->getKind() == attr::Final; }
};

class FlagEnumAttr : public InheritableAttr {
public:
  static FlagEnumAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FlagEnumAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FlagEnumAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FlagEnumAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FlagEnumAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static FlagEnumAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  FlagEnumAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::FlagEnum, false, false)
  {
  }

  FlagEnumAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::FlagEnum; }
};

class FlattenAttr : public InheritableAttr {
public:
  static FlattenAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FlattenAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FlattenAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FlattenAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FlattenAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static FlattenAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  FlattenAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Flatten, false, false)
  {
  }

  FlattenAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Flatten; }
};

class FormatAttr : public InheritableAttr {
IdentifierInfo * type;

int formatIdx;

int firstArg;

public:
  static FormatAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Type, int FormatIdx, int FirstArg, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FormatAttr(Ctx, CommonInfo, Type, FormatIdx, FirstArg);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FormatAttr *Create(ASTContext &Ctx, IdentifierInfo * Type, int FormatIdx, int FirstArg, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FormatAttr(Ctx, CommonInfo, Type, FormatIdx, FirstArg);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FormatAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Type, int FormatIdx, int FirstArg, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Type, FormatIdx, FirstArg, I);
  }
  static FormatAttr *Create(ASTContext &Ctx, IdentifierInfo * Type, int FormatIdx, int FirstArg, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Type, FormatIdx, FirstArg, I);
  }
  FormatAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * Type
              , int FormatIdx
              , int FirstArg
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Format, false, false)
              , type(Type)
              , formatIdx(FormatIdx)
              , firstArg(FirstArg)
  {
  }

  FormatAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getType() const {
    return type;
  }

  int getFormatIdx() const {
    return formatIdx;
  }

  int getFirstArg() const {
    return firstArg;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Format; }
};

class FormatArgAttr : public InheritableAttr {
ParamIdx formatIdx;

public:
  static FormatArgAttr *CreateImplicit(ASTContext &Ctx, ParamIdx FormatIdx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FormatArgAttr(Ctx, CommonInfo, FormatIdx);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FormatArgAttr *Create(ASTContext &Ctx, ParamIdx FormatIdx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) FormatArgAttr(Ctx, CommonInfo, FormatIdx);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static FormatArgAttr *CreateImplicit(ASTContext &Ctx, ParamIdx FormatIdx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, FormatIdx, I);
  }
  static FormatArgAttr *Create(ASTContext &Ctx, ParamIdx FormatIdx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, FormatIdx, I);
  }
  FormatArgAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ParamIdx FormatIdx
             )
    : InheritableAttr(Ctx, CommonInfo, attr::FormatArg, false, false)
              , formatIdx(FormatIdx)
  {
  }

  FormatArgAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  ParamIdx getFormatIdx() const {
    return formatIdx;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::FormatArg; }
};

class GNUInlineAttr : public InheritableAttr {
public:
  static GNUInlineAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) GNUInlineAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static GNUInlineAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) GNUInlineAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static GNUInlineAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static GNUInlineAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  GNUInlineAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::GNUInline, false, false)
  {
  }

  GNUInlineAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::GNUInline; }
};

class GuardedByAttr : public InheritableAttr {
Expr * arg;

public:
  static GuardedByAttr *CreateImplicit(ASTContext &Ctx, Expr * Arg, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) GuardedByAttr(Ctx, CommonInfo, Arg);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static GuardedByAttr *Create(ASTContext &Ctx, Expr * Arg, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) GuardedByAttr(Ctx, CommonInfo, Arg);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static GuardedByAttr *CreateImplicit(ASTContext &Ctx, Expr * Arg, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Arg, I);
  }
  static GuardedByAttr *Create(ASTContext &Ctx, Expr * Arg, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Arg, I);
  }
  GuardedByAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Arg
             )
    : InheritableAttr(Ctx, CommonInfo, attr::GuardedBy, true, true)
              , arg(Arg)
  {
  }

  GuardedByAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getArg() const {
    return arg;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::GuardedBy; }
};

class GuardedVarAttr : public InheritableAttr {
public:
  static GuardedVarAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) GuardedVarAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static GuardedVarAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) GuardedVarAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static GuardedVarAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static GuardedVarAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  GuardedVarAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::GuardedVar, false, false)
  {
  }

  GuardedVarAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::GuardedVar; }
};

class HIPPinnedShadowAttr : public InheritableAttr {
public:
  static HIPPinnedShadowAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) HIPPinnedShadowAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static HIPPinnedShadowAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) HIPPinnedShadowAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static HIPPinnedShadowAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static HIPPinnedShadowAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  HIPPinnedShadowAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::HIPPinnedShadow, false, false)
  {
  }

  HIPPinnedShadowAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::HIPPinnedShadow; }
};

class HotAttr : public InheritableAttr {
public:
  static HotAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) HotAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static HotAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) HotAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static HotAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static HotAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  HotAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Hot, false, false)
  {
  }

  HotAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Hot; }
};

class IBActionAttr : public InheritableAttr {
public:
  static IBActionAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IBActionAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IBActionAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IBActionAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IBActionAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static IBActionAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  IBActionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::IBAction, false, false)
  {
  }

  IBActionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::IBAction; }
};

class IBOutletAttr : public InheritableAttr {
public:
  static IBOutletAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IBOutletAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IBOutletAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IBOutletAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IBOutletAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static IBOutletAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  IBOutletAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::IBOutlet, false, false)
  {
  }

  IBOutletAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::IBOutlet; }
};

class IBOutletCollectionAttr : public InheritableAttr {
TypeSourceInfo * interface_;

public:
  static IBOutletCollectionAttr *CreateImplicit(ASTContext &Ctx, TypeSourceInfo * Interface, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IBOutletCollectionAttr(Ctx, CommonInfo, Interface);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IBOutletCollectionAttr *Create(ASTContext &Ctx, TypeSourceInfo * Interface, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IBOutletCollectionAttr(Ctx, CommonInfo, Interface);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IBOutletCollectionAttr *CreateImplicit(ASTContext &Ctx, TypeSourceInfo * Interface, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Interface, I);
  }
  static IBOutletCollectionAttr *Create(ASTContext &Ctx, TypeSourceInfo * Interface, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Interface, I);
  }
  IBOutletCollectionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , TypeSourceInfo * Interface
             )
    : InheritableAttr(Ctx, CommonInfo, attr::IBOutletCollection, false, false)
              , interface_(Interface)
  {
  }

  IBOutletCollectionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::IBOutletCollection, false, false)
              , interface_()
  {
  }

  IBOutletCollectionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  QualType getInterface() const {
    return interface_->getType();
  }  TypeSourceInfo * getInterfaceLoc() const {
    return interface_;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::IBOutletCollection; }
};

class IFuncAttr : public Attr {
unsigned resolverLength;
char *resolver;

public:
  static IFuncAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Resolver, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IFuncAttr(Ctx, CommonInfo, Resolver);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IFuncAttr *Create(ASTContext &Ctx, llvm::StringRef Resolver, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IFuncAttr(Ctx, CommonInfo, Resolver);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IFuncAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Resolver, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Resolver, I);
  }
  static IFuncAttr *Create(ASTContext &Ctx, llvm::StringRef Resolver, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Resolver, I);
  }
  IFuncAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Resolver
             )
    : Attr(Ctx, CommonInfo, attr::IFunc, false)
              , resolverLength(Resolver.size()),resolver(new (Ctx, 1) char[resolverLength])
  {
      if (!Resolver.empty())
        std::memcpy(resolver, Resolver.data(), resolverLength);
  }

  IFuncAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getResolver() const {
    return llvm::StringRef(resolver, resolverLength);
  }
  unsigned getResolverLength() const {
    return resolverLength;
  }
  void setResolver(ASTContext &C, llvm::StringRef S) {
    resolverLength = S.size();
    this->resolver = new (C, 1) char [resolverLength];
    if (!S.empty())
      std::memcpy(this->resolver, S.data(), resolverLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::IFunc; }
};

class InitPriorityAttr : public InheritableAttr {
unsigned priority;

public:
  static InitPriorityAttr *CreateImplicit(ASTContext &Ctx, unsigned Priority, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) InitPriorityAttr(Ctx, CommonInfo, Priority);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static InitPriorityAttr *Create(ASTContext &Ctx, unsigned Priority, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) InitPriorityAttr(Ctx, CommonInfo, Priority);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static InitPriorityAttr *CreateImplicit(ASTContext &Ctx, unsigned Priority, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Priority, I);
  }
  static InitPriorityAttr *Create(ASTContext &Ctx, unsigned Priority, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Priority, I);
  }
  InitPriorityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned Priority
             )
    : InheritableAttr(Ctx, CommonInfo, attr::InitPriority, false, false)
              , priority(Priority)
  {
  }

  InitPriorityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getPriority() const {
    return priority;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::InitPriority; }
};

class InitSegAttr : public Attr {
unsigned sectionLength;
char *section;

public:
  static InitSegAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Section, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) InitSegAttr(Ctx, CommonInfo, Section);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static InitSegAttr *Create(ASTContext &Ctx, llvm::StringRef Section, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) InitSegAttr(Ctx, CommonInfo, Section);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static InitSegAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Section, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Section, I);
  }
  static InitSegAttr *Create(ASTContext &Ctx, llvm::StringRef Section, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Section, I);
  }
  InitSegAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Section
             )
    : Attr(Ctx, CommonInfo, attr::InitSeg, false)
              , sectionLength(Section.size()),section(new (Ctx, 1) char[sectionLength])
  {
      if (!Section.empty())
        std::memcpy(section, Section.data(), sectionLength);
  }

  InitSegAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getSection() const {
    return llvm::StringRef(section, sectionLength);
  }
  unsigned getSectionLength() const {
    return sectionLength;
  }
  void setSection(ASTContext &C, llvm::StringRef S) {
    sectionLength = S.size();
    this->section = new (C, 1) char [sectionLength];
    if (!S.empty())
      std::memcpy(this->section, S.data(), sectionLength);
  }


  void printPrettyPragma(raw_ostream &OS, const PrintingPolicy &Policy) const {
    OS << " (" << getSection() << ')';
  }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::InitSeg; }
};

class IntelOclBiccAttr : public InheritableAttr {
public:
  static IntelOclBiccAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IntelOclBiccAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IntelOclBiccAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) IntelOclBiccAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static IntelOclBiccAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static IntelOclBiccAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  IntelOclBiccAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::IntelOclBicc, false, false)
  {
  }

  IntelOclBiccAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::IntelOclBicc; }
};

class InternalLinkageAttr : public InheritableAttr {
public:
  static InternalLinkageAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) InternalLinkageAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static InternalLinkageAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) InternalLinkageAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static InternalLinkageAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static InternalLinkageAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  InternalLinkageAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::InternalLinkage, false, false)
  {
  }

  InternalLinkageAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::InternalLinkage; }
};

class LTOVisibilityPublicAttr : public InheritableAttr {
public:
  static LTOVisibilityPublicAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LTOVisibilityPublicAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LTOVisibilityPublicAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LTOVisibilityPublicAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LTOVisibilityPublicAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static LTOVisibilityPublicAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  LTOVisibilityPublicAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::LTOVisibilityPublic, false, false)
  {
  }

  LTOVisibilityPublicAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::LTOVisibilityPublic; }
};

class LayoutVersionAttr : public InheritableAttr {
unsigned version;

public:
  static LayoutVersionAttr *CreateImplicit(ASTContext &Ctx, unsigned Version, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LayoutVersionAttr(Ctx, CommonInfo, Version);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LayoutVersionAttr *Create(ASTContext &Ctx, unsigned Version, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LayoutVersionAttr(Ctx, CommonInfo, Version);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LayoutVersionAttr *CreateImplicit(ASTContext &Ctx, unsigned Version, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Version, I);
  }
  static LayoutVersionAttr *Create(ASTContext &Ctx, unsigned Version, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Version, I);
  }
  LayoutVersionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned Version
             )
    : InheritableAttr(Ctx, CommonInfo, attr::LayoutVersion, false, false)
              , version(Version)
  {
  }

  LayoutVersionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getVersion() const {
    return version;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::LayoutVersion; }
};

class LifetimeBoundAttr : public InheritableAttr {
public:
  static LifetimeBoundAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LifetimeBoundAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LifetimeBoundAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LifetimeBoundAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LifetimeBoundAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static LifetimeBoundAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  LifetimeBoundAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::LifetimeBound, false, false)
  {
  }

  LifetimeBoundAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::LifetimeBound; }
};

class LockReturnedAttr : public InheritableAttr {
Expr * arg;

public:
  static LockReturnedAttr *CreateImplicit(ASTContext &Ctx, Expr * Arg, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LockReturnedAttr(Ctx, CommonInfo, Arg);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LockReturnedAttr *Create(ASTContext &Ctx, Expr * Arg, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LockReturnedAttr(Ctx, CommonInfo, Arg);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LockReturnedAttr *CreateImplicit(ASTContext &Ctx, Expr * Arg, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Arg, I);
  }
  static LockReturnedAttr *Create(ASTContext &Ctx, Expr * Arg, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Arg, I);
  }
  LockReturnedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Arg
             )
    : InheritableAttr(Ctx, CommonInfo, attr::LockReturned, true, false)
              , arg(Arg)
  {
  }

  LockReturnedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getArg() const {
    return arg;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::LockReturned; }
};

class LocksExcludedAttr : public InheritableAttr {
  unsigned args_Size;
  Expr * *args_;

public:
  static LocksExcludedAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LocksExcludedAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LocksExcludedAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LocksExcludedAttr(Ctx, CommonInfo, Args, ArgsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LocksExcludedAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static LocksExcludedAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Args, ArgsSize, I);
  }
  LocksExcludedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::LocksExcluded, true, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  LocksExcludedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::LocksExcluded, true, true)
              , args_Size(0), args_(nullptr)
  {
  }

  LocksExcludedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::LocksExcluded; }
};

class LoopHintAttr : public Attr {
public:
  enum OptionType {
    Vectorize,
    VectorizeWidth,
    Interleave,
    InterleaveCount,
    Unroll,
    UnrollCount,
    UnrollAndJam,
    UnrollAndJamCount,
    PipelineDisabled,
    PipelineInitiationInterval,
    Distribute,
    VectorizePredicate
  };
private:
  OptionType option;

public:
  enum LoopHintState {
    Enable,
    Disable,
    Numeric,
    AssumeSafety,
    Full
  };
private:
  LoopHintState state;

Expr * value;

public:
  enum Spelling {
    Pragma_clang_loop = 0,
    Pragma_unroll = 1,
    Pragma_nounroll = 2,
    Pragma_unroll_and_jam = 3,
    Pragma_nounroll_and_jam = 4,
  SpellingNotCalculated = 15

  };

  static LoopHintAttr *CreateImplicit(ASTContext &Ctx, OptionType Option, LoopHintState State, Expr * Value, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LoopHintAttr(Ctx, CommonInfo, Option, State, Value);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static LoopHintAttr *Create(ASTContext &Ctx, OptionType Option, LoopHintState State, Expr * Value, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) LoopHintAttr(Ctx, CommonInfo, Option, State, Value);
    return A;
  }

  static LoopHintAttr *CreateImplicit(ASTContext &Ctx, OptionType Option, LoopHintState State, Expr * Value, SourceRange Range, AttributeCommonInfo::Syntax Syntax, LoopHintAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Option, State, Value, I);
  }
  static LoopHintAttr *Create(ASTContext &Ctx, OptionType Option, LoopHintState State, Expr * Value, SourceRange Range, AttributeCommonInfo::Syntax Syntax, LoopHintAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Option, State, Value, I);
  }
  LoopHintAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , OptionType Option
              , LoopHintState State
              , Expr * Value
             )
    : Attr(Ctx, CommonInfo, attr::LoopHint, false)
              , option(Option)
              , state(State)
              , value(Value)
  {
  }

  LoopHintAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Pragma_clang_loop;
    case 1: return Pragma_unroll;
    case 2: return Pragma_nounroll;
    case 3: return Pragma_unroll_and_jam;
    case 4: return Pragma_nounroll_and_jam;
  }
  }
  OptionType getOption() const {
    return option;
  }

  static bool ConvertStrToOptionType(StringRef Val, OptionType &Out) {
    Optional<OptionType> R = llvm::StringSwitch<Optional<OptionType>>(Val)
      .Case("vectorize", LoopHintAttr::Vectorize)
      .Case("vectorize_width", LoopHintAttr::VectorizeWidth)
      .Case("interleave", LoopHintAttr::Interleave)
      .Case("interleave_count", LoopHintAttr::InterleaveCount)
      .Case("unroll", LoopHintAttr::Unroll)
      .Case("unroll_count", LoopHintAttr::UnrollCount)
      .Case("unroll_and_jam", LoopHintAttr::UnrollAndJam)
      .Case("unroll_and_jam_count", LoopHintAttr::UnrollAndJamCount)
      .Case("pipeline", LoopHintAttr::PipelineDisabled)
      .Case("pipeline_initiation_interval", LoopHintAttr::PipelineInitiationInterval)
      .Case("distribute", LoopHintAttr::Distribute)
      .Case("vectorize_predicate", LoopHintAttr::VectorizePredicate)
      .Default(Optional<OptionType>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertOptionTypeToStr(OptionType Val) {
    switch(Val) {
    case LoopHintAttr::Vectorize: return "vectorize";
    case LoopHintAttr::VectorizeWidth: return "vectorize_width";
    case LoopHintAttr::Interleave: return "interleave";
    case LoopHintAttr::InterleaveCount: return "interleave_count";
    case LoopHintAttr::Unroll: return "unroll";
    case LoopHintAttr::UnrollCount: return "unroll_count";
    case LoopHintAttr::UnrollAndJam: return "unroll_and_jam";
    case LoopHintAttr::UnrollAndJamCount: return "unroll_and_jam_count";
    case LoopHintAttr::PipelineDisabled: return "pipeline";
    case LoopHintAttr::PipelineInitiationInterval: return "pipeline_initiation_interval";
    case LoopHintAttr::Distribute: return "distribute";
    case LoopHintAttr::VectorizePredicate: return "vectorize_predicate";
    }
    llvm_unreachable("No enumerator with that value");
  }
  LoopHintState getState() const {
    return state;
  }

  static bool ConvertStrToLoopHintState(StringRef Val, LoopHintState &Out) {
    Optional<LoopHintState> R = llvm::StringSwitch<Optional<LoopHintState>>(Val)
      .Case("enable", LoopHintAttr::Enable)
      .Case("disable", LoopHintAttr::Disable)
      .Case("numeric", LoopHintAttr::Numeric)
      .Case("assume_safety", LoopHintAttr::AssumeSafety)
      .Case("full", LoopHintAttr::Full)
      .Default(Optional<LoopHintState>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertLoopHintStateToStr(LoopHintState Val) {
    switch(Val) {
    case LoopHintAttr::Enable: return "enable";
    case LoopHintAttr::Disable: return "disable";
    case LoopHintAttr::Numeric: return "numeric";
    case LoopHintAttr::AssumeSafety: return "assume_safety";
    case LoopHintAttr::Full: return "full";
    }
    llvm_unreachable("No enumerator with that value");
  }
  Expr * getValue() const {
    return value;
  }


  static const char *getOptionName(int Option) {
    switch(Option) {
    case Vectorize: return "vectorize";
    case VectorizeWidth: return "vectorize_width";
    case Interleave: return "interleave";
    case InterleaveCount: return "interleave_count";
    case Unroll: return "unroll";
    case UnrollCount: return "unroll_count";
    case UnrollAndJam: return "unroll_and_jam";
    case UnrollAndJamCount: return "unroll_and_jam_count";
    case PipelineDisabled: return "pipeline";
    case PipelineInitiationInterval: return "pipeline_initiation_interval";
    case Distribute: return "distribute";
    case VectorizePredicate: return "vectorize_predicate";
    }
    llvm_unreachable("Unhandled LoopHint option.");
  }

  void printPrettyPragma(raw_ostream &OS, const PrintingPolicy &Policy) const {
    unsigned SpellingIndex = getAttributeSpellingListIndex();
    // For "#pragma unroll" and "#pragma nounroll" the string "unroll" or
    // "nounroll" is already emitted as the pragma name.
    if (SpellingIndex == Pragma_nounroll || SpellingIndex == Pragma_nounroll_and_jam)
      return;
    else if (SpellingIndex == Pragma_unroll || SpellingIndex == Pragma_unroll_and_jam) {
      OS << ' ' << getValueString(Policy);
      return;
    }

    assert(SpellingIndex == Pragma_clang_loop && "Unexpected spelling");
    OS << ' ' << getOptionName(option) << getValueString(Policy);
  }

  // Return a string containing the loop hint argument including the
  // enclosing parentheses.
  std::string getValueString(const PrintingPolicy &Policy) const {
    std::string ValueName;
    llvm::raw_string_ostream OS(ValueName);
    OS << "(";
    if (state == Numeric)
      value->printPretty(OS, nullptr, Policy);
    else if (state == Enable)
      OS << "enable";
    else if (state == Full)
      OS << "full";
    else if (state == AssumeSafety)
      OS << "assume_safety";
    else
      OS << "disable";
    OS << ")";
    return OS.str();
  }

  // Return a string suitable for identifying this attribute in diagnostics.
  std::string getDiagnosticName(const PrintingPolicy &Policy) const {
    unsigned SpellingIndex = getAttributeSpellingListIndex();
    if (SpellingIndex == Pragma_nounroll)
      return "#pragma nounroll";
    else if (SpellingIndex == Pragma_unroll)
      return "#pragma unroll" + (option == UnrollCount ? getValueString(Policy) : "");
    else if (SpellingIndex == Pragma_nounroll_and_jam)
      return "#pragma nounroll_and_jam";
    else if (SpellingIndex == Pragma_unroll_and_jam)
      return "#pragma unroll_and_jam" +
        (option == UnrollAndJamCount ? getValueString(Policy) : "");

    assert(SpellingIndex == Pragma_clang_loop && "Unexpected spelling");
    return getOptionName(option) + getValueString(Policy);
  }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::LoopHint; }
};

class MIGServerRoutineAttr : public InheritableAttr {
public:
  static MIGServerRoutineAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MIGServerRoutineAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MIGServerRoutineAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MIGServerRoutineAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MIGServerRoutineAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static MIGServerRoutineAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  MIGServerRoutineAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MIGServerRoutine, false, false)
  {
  }

  MIGServerRoutineAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::MIGServerRoutine; }
};

class MSABIAttr : public InheritableAttr {
public:
  static MSABIAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSABIAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSABIAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSABIAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSABIAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static MSABIAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  MSABIAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MSABI, false, false)
  {
  }

  MSABIAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::MSABI; }
};

class MSAllocatorAttr : public InheritableAttr {
public:
  static MSAllocatorAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSAllocatorAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSAllocatorAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSAllocatorAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSAllocatorAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static MSAllocatorAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  MSAllocatorAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MSAllocator, false, false)
  {
  }

  MSAllocatorAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::MSAllocator; }
};

class MSInheritanceAttr : public InheritableAttr {
bool bestCase;

public:
  enum Spelling {
    Keyword_single_inheritance = 0,
    Keyword_multiple_inheritance = 1,
    Keyword_virtual_inheritance = 2,
    Keyword_unspecified_inheritance = 3,
  SpellingNotCalculated = 15

  };

  static MSInheritanceAttr *CreateImplicit(ASTContext &Ctx, bool BestCase, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSInheritanceAttr(Ctx, CommonInfo, BestCase);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSInheritanceAttr *Create(ASTContext &Ctx, bool BestCase, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSInheritanceAttr(Ctx, CommonInfo, BestCase);
    return A;
  }

  static MSInheritanceAttr *CreateImplicit(ASTContext &Ctx, bool BestCase, SourceRange Range, AttributeCommonInfo::Syntax Syntax, MSInheritanceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, BestCase, I);
  }
  static MSInheritanceAttr *Create(ASTContext &Ctx, bool BestCase, SourceRange Range, AttributeCommonInfo::Syntax Syntax, MSInheritanceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, BestCase, I);
  }
  static MSInheritanceAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSInheritanceAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSInheritanceAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSInheritanceAttr(Ctx, CommonInfo);
    return A;
  }

  static MSInheritanceAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, MSInheritanceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static MSInheritanceAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, MSInheritanceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  MSInheritanceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , bool BestCase
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MSInheritance, false, false)
              , bestCase(BestCase)
  {
  }

  MSInheritanceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MSInheritance, false, false)
              , bestCase()
  {
  }

  MSInheritanceAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Keyword_single_inheritance;
    case 1: return Keyword_multiple_inheritance;
    case 2: return Keyword_virtual_inheritance;
    case 3: return Keyword_unspecified_inheritance;
  }
  }
  bool getBestCase() const {
    return bestCase;
  }

  static const bool DefaultBestCase = true;


  MSInheritanceModel getInheritanceModel() const {
    // The spelling enum should agree with MSInheritanceModel.
    return MSInheritanceModel(getSemanticSpelling());
  }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::MSInheritance; }
};

class MSNoVTableAttr : public InheritableAttr {
public:
  static MSNoVTableAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSNoVTableAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSNoVTableAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSNoVTableAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSNoVTableAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static MSNoVTableAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  MSNoVTableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MSNoVTable, false, false)
  {
  }

  MSNoVTableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::MSNoVTable; }
};

class MSP430InterruptAttr : public InheritableAttr {
unsigned number;

public:
  static MSP430InterruptAttr *CreateImplicit(ASTContext &Ctx, unsigned Number, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSP430InterruptAttr(Ctx, CommonInfo, Number);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSP430InterruptAttr *Create(ASTContext &Ctx, unsigned Number, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSP430InterruptAttr(Ctx, CommonInfo, Number);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSP430InterruptAttr *CreateImplicit(ASTContext &Ctx, unsigned Number, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Number, I);
  }
  static MSP430InterruptAttr *Create(ASTContext &Ctx, unsigned Number, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Number, I);
  }
  MSP430InterruptAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned Number
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MSP430Interrupt, false, false)
              , number(Number)
  {
  }

  MSP430InterruptAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getNumber() const {
    return number;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::MSP430Interrupt; }
};

class MSStructAttr : public InheritableAttr {
public:
  static MSStructAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSStructAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSStructAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSStructAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSStructAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static MSStructAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  MSStructAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MSStruct, false, false)
  {
  }

  MSStructAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::MSStruct; }
};

class MSVtorDispAttr : public InheritableAttr {
unsigned vdm;

public:
  static MSVtorDispAttr *CreateImplicit(ASTContext &Ctx, unsigned Vdm, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSVtorDispAttr(Ctx, CommonInfo, Vdm);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSVtorDispAttr *Create(ASTContext &Ctx, unsigned Vdm, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MSVtorDispAttr(Ctx, CommonInfo, Vdm);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MSVtorDispAttr *CreateImplicit(ASTContext &Ctx, unsigned Vdm, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Vdm, I);
  }
  static MSVtorDispAttr *Create(ASTContext &Ctx, unsigned Vdm, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Vdm, I);
  }
  MSVtorDispAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned Vdm
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MSVtorDisp, false, false)
              , vdm(Vdm)
  {
  }

  MSVtorDispAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getVdm() const {
    return vdm;
  }


  MSVtorDispMode getVtorDispMode() const { return MSVtorDispMode(vdm); }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::MSVtorDisp; }
};

class MaxFieldAlignmentAttr : public InheritableAttr {
unsigned alignment;

public:
  static MaxFieldAlignmentAttr *CreateImplicit(ASTContext &Ctx, unsigned Alignment, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MaxFieldAlignmentAttr(Ctx, CommonInfo, Alignment);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MaxFieldAlignmentAttr *Create(ASTContext &Ctx, unsigned Alignment, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MaxFieldAlignmentAttr(Ctx, CommonInfo, Alignment);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MaxFieldAlignmentAttr *CreateImplicit(ASTContext &Ctx, unsigned Alignment, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Alignment, I);
  }
  static MaxFieldAlignmentAttr *Create(ASTContext &Ctx, unsigned Alignment, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Alignment, I);
  }
  MaxFieldAlignmentAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned Alignment
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MaxFieldAlignment, false, false)
              , alignment(Alignment)
  {
  }

  MaxFieldAlignmentAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getAlignment() const {
    return alignment;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::MaxFieldAlignment; }
};

class MayAliasAttr : public InheritableAttr {
public:
  static MayAliasAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MayAliasAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MayAliasAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MayAliasAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MayAliasAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static MayAliasAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  MayAliasAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MayAlias, false, false)
  {
  }

  MayAliasAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::MayAlias; }
};

class MicroMipsAttr : public InheritableAttr {
public:
  static MicroMipsAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MicroMipsAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MicroMipsAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MicroMipsAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MicroMipsAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static MicroMipsAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  MicroMipsAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MicroMips, false, false)
  {
  }

  MicroMipsAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::MicroMips; }
};

class MinSizeAttr : public InheritableAttr {
public:
  static MinSizeAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MinSizeAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MinSizeAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MinSizeAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MinSizeAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static MinSizeAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  MinSizeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MinSize, false, false)
  {
  }

  MinSizeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::MinSize; }
};

class MinVectorWidthAttr : public InheritableAttr {
unsigned vectorWidth;

public:
  static MinVectorWidthAttr *CreateImplicit(ASTContext &Ctx, unsigned VectorWidth, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MinVectorWidthAttr(Ctx, CommonInfo, VectorWidth);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MinVectorWidthAttr *Create(ASTContext &Ctx, unsigned VectorWidth, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MinVectorWidthAttr(Ctx, CommonInfo, VectorWidth);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MinVectorWidthAttr *CreateImplicit(ASTContext &Ctx, unsigned VectorWidth, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, VectorWidth, I);
  }
  static MinVectorWidthAttr *Create(ASTContext &Ctx, unsigned VectorWidth, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, VectorWidth, I);
  }
  MinVectorWidthAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned VectorWidth
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MinVectorWidth, false, false)
              , vectorWidth(VectorWidth)
  {
  }

  MinVectorWidthAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getVectorWidth() const {
    return vectorWidth;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::MinVectorWidth; }
};

class Mips16Attr : public InheritableAttr {
public:
  static Mips16Attr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) Mips16Attr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static Mips16Attr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) Mips16Attr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static Mips16Attr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static Mips16Attr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  Mips16Attr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Mips16, false, false)
  {
  }

  Mips16Attr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Mips16; }
};

class MipsInterruptAttr : public InheritableAttr {
public:
  enum InterruptType {
    sw0,
    sw1,
    hw0,
    hw1,
    hw2,
    hw3,
    hw4,
    hw5,
    eic
  };
private:
  InterruptType interrupt;

public:
  static MipsInterruptAttr *CreateImplicit(ASTContext &Ctx, InterruptType Interrupt, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MipsInterruptAttr(Ctx, CommonInfo, Interrupt);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MipsInterruptAttr *Create(ASTContext &Ctx, InterruptType Interrupt, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MipsInterruptAttr(Ctx, CommonInfo, Interrupt);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MipsInterruptAttr *CreateImplicit(ASTContext &Ctx, InterruptType Interrupt, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Interrupt, I);
  }
  static MipsInterruptAttr *Create(ASTContext &Ctx, InterruptType Interrupt, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Interrupt, I);
  }
  MipsInterruptAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , InterruptType Interrupt
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MipsInterrupt, false, false)
              , interrupt(Interrupt)
  {
  }

  MipsInterruptAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  InterruptType getInterrupt() const {
    return interrupt;
  }

  static bool ConvertStrToInterruptType(StringRef Val, InterruptType &Out) {
    Optional<InterruptType> R = llvm::StringSwitch<Optional<InterruptType>>(Val)
      .Case("vector=sw0", MipsInterruptAttr::sw0)
      .Case("vector=sw1", MipsInterruptAttr::sw1)
      .Case("vector=hw0", MipsInterruptAttr::hw0)
      .Case("vector=hw1", MipsInterruptAttr::hw1)
      .Case("vector=hw2", MipsInterruptAttr::hw2)
      .Case("vector=hw3", MipsInterruptAttr::hw3)
      .Case("vector=hw4", MipsInterruptAttr::hw4)
      .Case("vector=hw5", MipsInterruptAttr::hw5)
      .Case("eic", MipsInterruptAttr::eic)
      .Case("", MipsInterruptAttr::eic)
      .Default(Optional<InterruptType>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertInterruptTypeToStr(InterruptType Val) {
    switch(Val) {
    case MipsInterruptAttr::sw0: return "vector=sw0";
    case MipsInterruptAttr::sw1: return "vector=sw1";
    case MipsInterruptAttr::hw0: return "vector=hw0";
    case MipsInterruptAttr::hw1: return "vector=hw1";
    case MipsInterruptAttr::hw2: return "vector=hw2";
    case MipsInterruptAttr::hw3: return "vector=hw3";
    case MipsInterruptAttr::hw4: return "vector=hw4";
    case MipsInterruptAttr::hw5: return "vector=hw5";
    case MipsInterruptAttr::eic: return "eic";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::MipsInterrupt; }
};

class MipsLongCallAttr : public InheritableAttr {
public:
  enum Spelling {
    GNU_long_call = 0,
    CXX11_gnu_long_call = 1,
    GNU_far = 2,
    CXX11_gnu_far = 3,
  SpellingNotCalculated = 15

  };

  static MipsLongCallAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MipsLongCallAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MipsLongCallAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MipsLongCallAttr(Ctx, CommonInfo);
    return A;
  }

  static MipsLongCallAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, MipsLongCallAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static MipsLongCallAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, MipsLongCallAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  MipsLongCallAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MipsLongCall, false, false)
  {
  }

  MipsLongCallAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_long_call;
    case 1: return CXX11_gnu_long_call;
    case 2: return GNU_far;
    case 3: return CXX11_gnu_far;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::MipsLongCall; }
};

class MipsShortCallAttr : public InheritableAttr {
public:
  enum Spelling {
    GNU_short_call = 0,
    CXX11_gnu_short_call = 1,
    GNU_near = 2,
    CXX11_gnu_near = 3,
  SpellingNotCalculated = 15

  };

  static MipsShortCallAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MipsShortCallAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static MipsShortCallAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) MipsShortCallAttr(Ctx, CommonInfo);
    return A;
  }

  static MipsShortCallAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, MipsShortCallAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static MipsShortCallAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, MipsShortCallAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  MipsShortCallAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::MipsShortCall, false, false)
  {
  }

  MipsShortCallAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_short_call;
    case 1: return CXX11_gnu_short_call;
    case 2: return GNU_near;
    case 3: return CXX11_gnu_near;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::MipsShortCall; }
};

class ModeAttr : public Attr {
IdentifierInfo * mode;

public:
  static ModeAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Mode, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ModeAttr(Ctx, CommonInfo, Mode);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ModeAttr *Create(ASTContext &Ctx, IdentifierInfo * Mode, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ModeAttr(Ctx, CommonInfo, Mode);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ModeAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Mode, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Mode, I);
  }
  static ModeAttr *Create(ASTContext &Ctx, IdentifierInfo * Mode, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Mode, I);
  }
  ModeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * Mode
             )
    : Attr(Ctx, CommonInfo, attr::Mode, false)
              , mode(Mode)
  {
  }

  ModeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getMode() const {
    return mode;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Mode; }
};

class NSConsumedAttr : public InheritableParamAttr {
public:
  static NSConsumedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSConsumedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSConsumedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSConsumedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSConsumedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NSConsumedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NSConsumedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::NSConsumed, false, false)
  {
  }

  NSConsumedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NSConsumed; }
};

class NSConsumesSelfAttr : public InheritableAttr {
public:
  static NSConsumesSelfAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSConsumesSelfAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSConsumesSelfAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSConsumesSelfAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSConsumesSelfAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NSConsumesSelfAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NSConsumesSelfAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NSConsumesSelf, false, false)
  {
  }

  NSConsumesSelfAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NSConsumesSelf; }
};

class NSReturnsAutoreleasedAttr : public InheritableAttr {
public:
  static NSReturnsAutoreleasedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSReturnsAutoreleasedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSReturnsAutoreleasedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSReturnsAutoreleasedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSReturnsAutoreleasedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NSReturnsAutoreleasedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NSReturnsAutoreleasedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NSReturnsAutoreleased, false, false)
  {
  }

  NSReturnsAutoreleasedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NSReturnsAutoreleased; }
};

class NSReturnsNotRetainedAttr : public InheritableAttr {
public:
  static NSReturnsNotRetainedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSReturnsNotRetainedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSReturnsNotRetainedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSReturnsNotRetainedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSReturnsNotRetainedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NSReturnsNotRetainedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NSReturnsNotRetainedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NSReturnsNotRetained, false, false)
  {
  }

  NSReturnsNotRetainedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NSReturnsNotRetained; }
};

class NSReturnsRetainedAttr : public InheritableAttr {
public:
  static NSReturnsRetainedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSReturnsRetainedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSReturnsRetainedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NSReturnsRetainedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NSReturnsRetainedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NSReturnsRetainedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NSReturnsRetainedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NSReturnsRetained, false, false)
  {
  }

  NSReturnsRetainedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NSReturnsRetained; }
};

class NakedAttr : public InheritableAttr {
public:
  static NakedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NakedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NakedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NakedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NakedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NakedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NakedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Naked, false, false)
  {
  }

  NakedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Naked; }
};

class NoAliasAttr : public InheritableAttr {
public:
  static NoAliasAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoAliasAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoAliasAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoAliasAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoAliasAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoAliasAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoAliasAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoAlias, false, false)
  {
  }

  NoAliasAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoAlias; }
};

class NoBuiltinAttr : public Attr {
  unsigned builtinNames_Size;
  StringRef *builtinNames_;

public:
  static NoBuiltinAttr *CreateImplicit(ASTContext &Ctx, StringRef *BuiltinNames, unsigned BuiltinNamesSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoBuiltinAttr(Ctx, CommonInfo, BuiltinNames, BuiltinNamesSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoBuiltinAttr *Create(ASTContext &Ctx, StringRef *BuiltinNames, unsigned BuiltinNamesSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoBuiltinAttr(Ctx, CommonInfo, BuiltinNames, BuiltinNamesSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoBuiltinAttr *CreateImplicit(ASTContext &Ctx, StringRef *BuiltinNames, unsigned BuiltinNamesSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, BuiltinNames, BuiltinNamesSize, I);
  }
  static NoBuiltinAttr *Create(ASTContext &Ctx, StringRef *BuiltinNames, unsigned BuiltinNamesSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, BuiltinNames, BuiltinNamesSize, I);
  }
  NoBuiltinAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , StringRef *BuiltinNames, unsigned BuiltinNamesSize
             )
    : Attr(Ctx, CommonInfo, attr::NoBuiltin, false)
              , builtinNames_Size(BuiltinNamesSize), builtinNames_(new (Ctx, 16) StringRef[builtinNames_Size])
  {
    for (size_t I = 0, E = builtinNames_Size; I != E;
         ++I) {
      StringRef Ref = BuiltinNames[I];
      if (!Ref.empty()) {
        char *Mem = new (Ctx, 1) char[Ref.size()];
        std::memcpy(Mem, Ref.data(), Ref.size());
        builtinNames_[I] = StringRef(Mem, Ref.size());
      }
    }
  }

  NoBuiltinAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::NoBuiltin, false)
              , builtinNames_Size(0), builtinNames_(nullptr)
  {
  }

  NoBuiltinAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef StringRef* builtinNames_iterator;
  builtinNames_iterator builtinNames_begin() const { return builtinNames_; }
  builtinNames_iterator builtinNames_end() const { return builtinNames_ + builtinNames_Size; }
  unsigned builtinNames_size() const { return builtinNames_Size; }
  llvm::iterator_range<builtinNames_iterator> builtinNames() const { return llvm::make_range(builtinNames_begin(), builtinNames_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::NoBuiltin; }
};

class NoCommonAttr : public InheritableAttr {
public:
  static NoCommonAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoCommonAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoCommonAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoCommonAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoCommonAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoCommonAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoCommonAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoCommon, false, false)
  {
  }

  NoCommonAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoCommon; }
};

class NoDebugAttr : public InheritableAttr {
public:
  static NoDebugAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoDebugAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoDebugAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoDebugAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoDebugAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoDebugAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoDebugAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoDebug, false, false)
  {
  }

  NoDebugAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoDebug; }
};

class NoDerefAttr : public TypeAttr {
public:
  static NoDerefAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoDerefAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoDerefAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoDerefAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoDerefAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoDerefAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoDerefAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::NoDeref, false)
  {
  }

  NoDerefAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoDeref; }
};

class NoDestroyAttr : public InheritableAttr {
public:
  static NoDestroyAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoDestroyAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoDestroyAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoDestroyAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoDestroyAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoDestroyAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoDestroyAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoDestroy, false, false)
  {
  }

  NoDestroyAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoDestroy; }
};

class NoDuplicateAttr : public InheritableAttr {
public:
  static NoDuplicateAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoDuplicateAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoDuplicateAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoDuplicateAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoDuplicateAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoDuplicateAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoDuplicateAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoDuplicate, false, false)
  {
  }

  NoDuplicateAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoDuplicate; }
};

class NoEscapeAttr : public Attr {
public:
  static NoEscapeAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoEscapeAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoEscapeAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoEscapeAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoEscapeAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoEscapeAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoEscapeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::NoEscape, false)
  {
  }

  NoEscapeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoEscape; }
};

class NoInlineAttr : public InheritableAttr {
public:
  static NoInlineAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoInlineAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoInlineAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoInlineAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoInlineAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoInlineAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoInlineAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoInline, false, false)
  {
  }

  NoInlineAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoInline; }
};

class NoInstrumentFunctionAttr : public InheritableAttr {
public:
  static NoInstrumentFunctionAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoInstrumentFunctionAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoInstrumentFunctionAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoInstrumentFunctionAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoInstrumentFunctionAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoInstrumentFunctionAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoInstrumentFunctionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoInstrumentFunction, false, false)
  {
  }

  NoInstrumentFunctionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoInstrumentFunction; }
};

class NoMicroMipsAttr : public InheritableAttr {
public:
  static NoMicroMipsAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoMicroMipsAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoMicroMipsAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoMicroMipsAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoMicroMipsAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoMicroMipsAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoMicroMipsAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoMicroMips, false, false)
  {
  }

  NoMicroMipsAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoMicroMips; }
};

class NoMips16Attr : public InheritableAttr {
public:
  static NoMips16Attr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoMips16Attr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoMips16Attr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoMips16Attr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoMips16Attr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoMips16Attr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoMips16Attr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoMips16, false, false)
  {
  }

  NoMips16Attr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoMips16; }
};

class NoReturnAttr : public InheritableAttr {
public:
  static NoReturnAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoReturnAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoReturnAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoReturnAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoReturnAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoReturnAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoReturnAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoReturn, false, false)
  {
  }

  NoReturnAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoReturn; }
};

class NoSanitizeAttr : public InheritableAttr {
  unsigned sanitizers_Size;
  StringRef *sanitizers_;

public:
  static NoSanitizeAttr *CreateImplicit(ASTContext &Ctx, StringRef *Sanitizers, unsigned SanitizersSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoSanitizeAttr(Ctx, CommonInfo, Sanitizers, SanitizersSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoSanitizeAttr *Create(ASTContext &Ctx, StringRef *Sanitizers, unsigned SanitizersSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoSanitizeAttr(Ctx, CommonInfo, Sanitizers, SanitizersSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoSanitizeAttr *CreateImplicit(ASTContext &Ctx, StringRef *Sanitizers, unsigned SanitizersSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Sanitizers, SanitizersSize, I);
  }
  static NoSanitizeAttr *Create(ASTContext &Ctx, StringRef *Sanitizers, unsigned SanitizersSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Sanitizers, SanitizersSize, I);
  }
  NoSanitizeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , StringRef *Sanitizers, unsigned SanitizersSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoSanitize, false, false)
              , sanitizers_Size(SanitizersSize), sanitizers_(new (Ctx, 16) StringRef[sanitizers_Size])
  {
    for (size_t I = 0, E = sanitizers_Size; I != E;
         ++I) {
      StringRef Ref = Sanitizers[I];
      if (!Ref.empty()) {
        char *Mem = new (Ctx, 1) char[Ref.size()];
        std::memcpy(Mem, Ref.data(), Ref.size());
        sanitizers_[I] = StringRef(Mem, Ref.size());
      }
    }
  }

  NoSanitizeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoSanitize, false, false)
              , sanitizers_Size(0), sanitizers_(nullptr)
  {
  }

  NoSanitizeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef StringRef* sanitizers_iterator;
  sanitizers_iterator sanitizers_begin() const { return sanitizers_; }
  sanitizers_iterator sanitizers_end() const { return sanitizers_ + sanitizers_Size; }
  unsigned sanitizers_size() const { return sanitizers_Size; }
  llvm::iterator_range<sanitizers_iterator> sanitizers() const { return llvm::make_range(sanitizers_begin(), sanitizers_end()); }



    SanitizerMask getMask() const {
      SanitizerMask Mask;
      for (auto SanitizerName : sanitizers()) {
        SanitizerMask ParsedMask =
            parseSanitizerValue(SanitizerName, /*AllowGroups=*/true);
        Mask |= expandSanitizerGroups(ParsedMask);
      }
      return Mask;
    }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::NoSanitize; }
};

class NoSpeculativeLoadHardeningAttr : public InheritableAttr {
public:
  static NoSpeculativeLoadHardeningAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoSpeculativeLoadHardeningAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoSpeculativeLoadHardeningAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoSpeculativeLoadHardeningAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoSpeculativeLoadHardeningAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoSpeculativeLoadHardeningAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoSpeculativeLoadHardeningAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoSpeculativeLoadHardening, false, false)
  {
  }

  NoSpeculativeLoadHardeningAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoSpeculativeLoadHardening; }
};

class NoSplitStackAttr : public InheritableAttr {
public:
  static NoSplitStackAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoSplitStackAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoSplitStackAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoSplitStackAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoSplitStackAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoSplitStackAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoSplitStackAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoSplitStack, false, false)
  {
  }

  NoSplitStackAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoSplitStack; }
};

class NoStackProtectorAttr : public InheritableAttr {
public:
  static NoStackProtectorAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoStackProtectorAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoStackProtectorAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoStackProtectorAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoStackProtectorAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoStackProtectorAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoStackProtectorAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoStackProtector, false, false)
  {
  }

  NoStackProtectorAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoStackProtector; }
};

class NoThreadSafetyAnalysisAttr : public InheritableAttr {
public:
  static NoThreadSafetyAnalysisAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoThreadSafetyAnalysisAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoThreadSafetyAnalysisAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoThreadSafetyAnalysisAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoThreadSafetyAnalysisAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoThreadSafetyAnalysisAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoThreadSafetyAnalysisAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoThreadSafetyAnalysis, false, false)
  {
  }

  NoThreadSafetyAnalysisAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoThreadSafetyAnalysis; }
};

class NoThrowAttr : public InheritableAttr {
public:
  static NoThrowAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoThrowAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoThrowAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoThrowAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoThrowAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoThrowAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoThrowAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoThrow, false, false)
  {
  }

  NoThrowAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoThrow; }
};

class NoUniqueAddressAttr : public InheritableAttr {
public:
  static NoUniqueAddressAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoUniqueAddressAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoUniqueAddressAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NoUniqueAddressAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NoUniqueAddressAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NoUniqueAddressAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NoUniqueAddressAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NoUniqueAddress, false, false)
  {
  }

  NoUniqueAddressAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NoUniqueAddress; }
};

class NonNullAttr : public InheritableParamAttr {
  unsigned args_Size;
  ParamIdx *args_;

public:
  static NonNullAttr *CreateImplicit(ASTContext &Ctx, ParamIdx *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NonNullAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NonNullAttr *Create(ASTContext &Ctx, ParamIdx *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NonNullAttr(Ctx, CommonInfo, Args, ArgsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NonNullAttr *CreateImplicit(ASTContext &Ctx, ParamIdx *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static NonNullAttr *Create(ASTContext &Ctx, ParamIdx *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Args, ArgsSize, I);
  }
  NonNullAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ParamIdx *Args, unsigned ArgsSize
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::NonNull, false, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) ParamIdx[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  NonNullAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::NonNull, false, true)
              , args_Size(0), args_(nullptr)
  {
  }

  NonNullAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef ParamIdx* args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }



    bool isNonNull(unsigned IdxAST) const {
      if (!args_size())
        return true;
      return args_end() != std::find_if(
          args_begin(), args_end(),
          [=](const ParamIdx &Idx) { return Idx.getASTIndex() == IdxAST; });
    }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::NonNull; }
};

class NotTailCalledAttr : public InheritableAttr {
public:
  static NotTailCalledAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NotTailCalledAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NotTailCalledAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) NotTailCalledAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static NotTailCalledAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static NotTailCalledAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  NotTailCalledAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::NotTailCalled, false, false)
  {
  }

  NotTailCalledAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::NotTailCalled; }
};

class OMPAllocateDeclAttr : public InheritableAttr {
public:
  enum AllocatorTypeTy {
    OMPDefaultMemAlloc,
    OMPLargeCapMemAlloc,
    OMPConstMemAlloc,
    OMPHighBWMemAlloc,
    OMPLowLatMemAlloc,
    OMPCGroupMemAlloc,
    OMPPTeamMemAlloc,
    OMPThreadMemAlloc,
    OMPUserDefinedMemAlloc
  };
private:
  AllocatorTypeTy allocatorType;

Expr * allocator;

public:
  static OMPAllocateDeclAttr *CreateImplicit(ASTContext &Ctx, AllocatorTypeTy AllocatorType, Expr * Allocator, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPAllocateDeclAttr(Ctx, CommonInfo, AllocatorType, Allocator);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPAllocateDeclAttr *Create(ASTContext &Ctx, AllocatorTypeTy AllocatorType, Expr * Allocator, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPAllocateDeclAttr(Ctx, CommonInfo, AllocatorType, Allocator);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPAllocateDeclAttr *CreateImplicit(ASTContext &Ctx, AllocatorTypeTy AllocatorType, Expr * Allocator, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, AllocatorType, Allocator, I);
  }
  static OMPAllocateDeclAttr *Create(ASTContext &Ctx, AllocatorTypeTy AllocatorType, Expr * Allocator, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, AllocatorType, Allocator, I);
  }
  OMPAllocateDeclAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , AllocatorTypeTy AllocatorType
              , Expr * Allocator
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OMPAllocateDecl, false, false)
              , allocatorType(AllocatorType)
              , allocator(Allocator)
  {
  }

  OMPAllocateDeclAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  AllocatorTypeTy getAllocatorType() const {
    return allocatorType;
  }

  static bool ConvertStrToAllocatorTypeTy(StringRef Val, AllocatorTypeTy &Out) {
    Optional<AllocatorTypeTy> R = llvm::StringSwitch<Optional<AllocatorTypeTy>>(Val)
      .Case("omp_default_mem_alloc", OMPAllocateDeclAttr::OMPDefaultMemAlloc)
      .Case("omp_large_cap_mem_alloc", OMPAllocateDeclAttr::OMPLargeCapMemAlloc)
      .Case("omp_const_mem_alloc", OMPAllocateDeclAttr::OMPConstMemAlloc)
      .Case("omp_high_bw_mem_alloc", OMPAllocateDeclAttr::OMPHighBWMemAlloc)
      .Case("omp_low_lat_mem_alloc", OMPAllocateDeclAttr::OMPLowLatMemAlloc)
      .Case("omp_cgroup_mem_alloc", OMPAllocateDeclAttr::OMPCGroupMemAlloc)
      .Case("omp_pteam_mem_alloc", OMPAllocateDeclAttr::OMPPTeamMemAlloc)
      .Case("omp_thread_mem_alloc", OMPAllocateDeclAttr::OMPThreadMemAlloc)
      .Case("", OMPAllocateDeclAttr::OMPUserDefinedMemAlloc)
      .Default(Optional<AllocatorTypeTy>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertAllocatorTypeTyToStr(AllocatorTypeTy Val) {
    switch(Val) {
    case OMPAllocateDeclAttr::OMPDefaultMemAlloc: return "omp_default_mem_alloc";
    case OMPAllocateDeclAttr::OMPLargeCapMemAlloc: return "omp_large_cap_mem_alloc";
    case OMPAllocateDeclAttr::OMPConstMemAlloc: return "omp_const_mem_alloc";
    case OMPAllocateDeclAttr::OMPHighBWMemAlloc: return "omp_high_bw_mem_alloc";
    case OMPAllocateDeclAttr::OMPLowLatMemAlloc: return "omp_low_lat_mem_alloc";
    case OMPAllocateDeclAttr::OMPCGroupMemAlloc: return "omp_cgroup_mem_alloc";
    case OMPAllocateDeclAttr::OMPPTeamMemAlloc: return "omp_pteam_mem_alloc";
    case OMPAllocateDeclAttr::OMPThreadMemAlloc: return "omp_thread_mem_alloc";
    case OMPAllocateDeclAttr::OMPUserDefinedMemAlloc: return "";
    }
    llvm_unreachable("No enumerator with that value");
  }
  Expr * getAllocator() const {
    return allocator;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::OMPAllocateDecl; }
};

class OMPCaptureKindAttr : public Attr {
unsigned captureKind;

public:
  static OMPCaptureKindAttr *CreateImplicit(ASTContext &Ctx, unsigned CaptureKind, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPCaptureKindAttr(Ctx, CommonInfo, CaptureKind);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPCaptureKindAttr *Create(ASTContext &Ctx, unsigned CaptureKind, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPCaptureKindAttr(Ctx, CommonInfo, CaptureKind);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPCaptureKindAttr *CreateImplicit(ASTContext &Ctx, unsigned CaptureKind, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, CaptureKind, I);
  }
  static OMPCaptureKindAttr *Create(ASTContext &Ctx, unsigned CaptureKind, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, CaptureKind, I);
  }
  OMPCaptureKindAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned CaptureKind
             )
    : Attr(Ctx, CommonInfo, attr::OMPCaptureKind, false)
              , captureKind(CaptureKind)
  {
  }

  OMPCaptureKindAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getCaptureKind() const {
    return captureKind;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::OMPCaptureKind; }
};

class OMPCaptureNoInitAttr : public InheritableAttr {
public:
  static OMPCaptureNoInitAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPCaptureNoInitAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPCaptureNoInitAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPCaptureNoInitAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPCaptureNoInitAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OMPCaptureNoInitAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OMPCaptureNoInitAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OMPCaptureNoInit, false, false)
  {
  }

  OMPCaptureNoInitAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OMPCaptureNoInit; }
};

class OMPDeclareSimdDeclAttr : public Attr {
public:
  enum BranchStateTy {
    BS_Undefined,
    BS_Inbranch,
    BS_Notinbranch
  };
private:
  BranchStateTy branchState;

Expr * simdlen;

  unsigned uniforms_Size;
  Expr * *uniforms_;

  unsigned aligneds_Size;
  Expr * *aligneds_;

  unsigned alignments_Size;
  Expr * *alignments_;

  unsigned linears_Size;
  Expr * *linears_;

  unsigned modifiers_Size;
  unsigned *modifiers_;

  unsigned steps_Size;
  Expr * *steps_;

public:
  static OMPDeclareSimdDeclAttr *CreateImplicit(ASTContext &Ctx, BranchStateTy BranchState, Expr * Simdlen, Expr * *Uniforms, unsigned UniformsSize, Expr * *Aligneds, unsigned AlignedsSize, Expr * *Alignments, unsigned AlignmentsSize, Expr * *Linears, unsigned LinearsSize, unsigned *Modifiers, unsigned ModifiersSize, Expr * *Steps, unsigned StepsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPDeclareSimdDeclAttr(Ctx, CommonInfo, BranchState, Simdlen, Uniforms, UniformsSize, Aligneds, AlignedsSize, Alignments, AlignmentsSize, Linears, LinearsSize, Modifiers, ModifiersSize, Steps, StepsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPDeclareSimdDeclAttr *Create(ASTContext &Ctx, BranchStateTy BranchState, Expr * Simdlen, Expr * *Uniforms, unsigned UniformsSize, Expr * *Aligneds, unsigned AlignedsSize, Expr * *Alignments, unsigned AlignmentsSize, Expr * *Linears, unsigned LinearsSize, unsigned *Modifiers, unsigned ModifiersSize, Expr * *Steps, unsigned StepsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPDeclareSimdDeclAttr(Ctx, CommonInfo, BranchState, Simdlen, Uniforms, UniformsSize, Aligneds, AlignedsSize, Alignments, AlignmentsSize, Linears, LinearsSize, Modifiers, ModifiersSize, Steps, StepsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPDeclareSimdDeclAttr *CreateImplicit(ASTContext &Ctx, BranchStateTy BranchState, Expr * Simdlen, Expr * *Uniforms, unsigned UniformsSize, Expr * *Aligneds, unsigned AlignedsSize, Expr * *Alignments, unsigned AlignmentsSize, Expr * *Linears, unsigned LinearsSize, unsigned *Modifiers, unsigned ModifiersSize, Expr * *Steps, unsigned StepsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, BranchState, Simdlen, Uniforms, UniformsSize, Aligneds, AlignedsSize, Alignments, AlignmentsSize, Linears, LinearsSize, Modifiers, ModifiersSize, Steps, StepsSize, I);
  }
  static OMPDeclareSimdDeclAttr *Create(ASTContext &Ctx, BranchStateTy BranchState, Expr * Simdlen, Expr * *Uniforms, unsigned UniformsSize, Expr * *Aligneds, unsigned AlignedsSize, Expr * *Alignments, unsigned AlignmentsSize, Expr * *Linears, unsigned LinearsSize, unsigned *Modifiers, unsigned ModifiersSize, Expr * *Steps, unsigned StepsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, BranchState, Simdlen, Uniforms, UniformsSize, Aligneds, AlignedsSize, Alignments, AlignmentsSize, Linears, LinearsSize, Modifiers, ModifiersSize, Steps, StepsSize, I);
  }
  OMPDeclareSimdDeclAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , BranchStateTy BranchState
              , Expr * Simdlen
              , Expr * *Uniforms, unsigned UniformsSize
              , Expr * *Aligneds, unsigned AlignedsSize
              , Expr * *Alignments, unsigned AlignmentsSize
              , Expr * *Linears, unsigned LinearsSize
              , unsigned *Modifiers, unsigned ModifiersSize
              , Expr * *Steps, unsigned StepsSize
             )
    : Attr(Ctx, CommonInfo, attr::OMPDeclareSimdDecl, false)
              , branchState(BranchState)
              , simdlen(Simdlen)
              , uniforms_Size(UniformsSize), uniforms_(new (Ctx, 16) Expr *[uniforms_Size])
              , aligneds_Size(AlignedsSize), aligneds_(new (Ctx, 16) Expr *[aligneds_Size])
              , alignments_Size(AlignmentsSize), alignments_(new (Ctx, 16) Expr *[alignments_Size])
              , linears_Size(LinearsSize), linears_(new (Ctx, 16) Expr *[linears_Size])
              , modifiers_Size(ModifiersSize), modifiers_(new (Ctx, 16) unsigned[modifiers_Size])
              , steps_Size(StepsSize), steps_(new (Ctx, 16) Expr *[steps_Size])
  {
    std::copy(Uniforms, Uniforms + uniforms_Size, uniforms_);
    std::copy(Aligneds, Aligneds + aligneds_Size, aligneds_);
    std::copy(Alignments, Alignments + alignments_Size, alignments_);
    std::copy(Linears, Linears + linears_Size, linears_);
    std::copy(Modifiers, Modifiers + modifiers_Size, modifiers_);
    std::copy(Steps, Steps + steps_Size, steps_);
  }

  OMPDeclareSimdDeclAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , BranchStateTy BranchState
              , Expr * Simdlen
             )
    : Attr(Ctx, CommonInfo, attr::OMPDeclareSimdDecl, false)
              , branchState(BranchState)
              , simdlen(Simdlen)
              , uniforms_Size(0), uniforms_(nullptr)
              , aligneds_Size(0), aligneds_(nullptr)
              , alignments_Size(0), alignments_(nullptr)
              , linears_Size(0), linears_(nullptr)
              , modifiers_Size(0), modifiers_(nullptr)
              , steps_Size(0), steps_(nullptr)
  {
  }

  OMPDeclareSimdDeclAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  BranchStateTy getBranchState() const {
    return branchState;
  }

  static bool ConvertStrToBranchStateTy(StringRef Val, BranchStateTy &Out) {
    Optional<BranchStateTy> R = llvm::StringSwitch<Optional<BranchStateTy>>(Val)
      .Case("", OMPDeclareSimdDeclAttr::BS_Undefined)
      .Case("inbranch", OMPDeclareSimdDeclAttr::BS_Inbranch)
      .Case("notinbranch", OMPDeclareSimdDeclAttr::BS_Notinbranch)
      .Default(Optional<BranchStateTy>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertBranchStateTyToStr(BranchStateTy Val) {
    switch(Val) {
    case OMPDeclareSimdDeclAttr::BS_Undefined: return "";
    case OMPDeclareSimdDeclAttr::BS_Inbranch: return "inbranch";
    case OMPDeclareSimdDeclAttr::BS_Notinbranch: return "notinbranch";
    }
    llvm_unreachable("No enumerator with that value");
  }
  Expr * getSimdlen() const {
    return simdlen;
  }

  typedef Expr ** uniforms_iterator;
  uniforms_iterator uniforms_begin() const { return uniforms_; }
  uniforms_iterator uniforms_end() const { return uniforms_ + uniforms_Size; }
  unsigned uniforms_size() const { return uniforms_Size; }
  llvm::iterator_range<uniforms_iterator> uniforms() const { return llvm::make_range(uniforms_begin(), uniforms_end()); }


  typedef Expr ** aligneds_iterator;
  aligneds_iterator aligneds_begin() const { return aligneds_; }
  aligneds_iterator aligneds_end() const { return aligneds_ + aligneds_Size; }
  unsigned aligneds_size() const { return aligneds_Size; }
  llvm::iterator_range<aligneds_iterator> aligneds() const { return llvm::make_range(aligneds_begin(), aligneds_end()); }


  typedef Expr ** alignments_iterator;
  alignments_iterator alignments_begin() const { return alignments_; }
  alignments_iterator alignments_end() const { return alignments_ + alignments_Size; }
  unsigned alignments_size() const { return alignments_Size; }
  llvm::iterator_range<alignments_iterator> alignments() const { return llvm::make_range(alignments_begin(), alignments_end()); }


  typedef Expr ** linears_iterator;
  linears_iterator linears_begin() const { return linears_; }
  linears_iterator linears_end() const { return linears_ + linears_Size; }
  unsigned linears_size() const { return linears_Size; }
  llvm::iterator_range<linears_iterator> linears() const { return llvm::make_range(linears_begin(), linears_end()); }


  typedef unsigned* modifiers_iterator;
  modifiers_iterator modifiers_begin() const { return modifiers_; }
  modifiers_iterator modifiers_end() const { return modifiers_ + modifiers_Size; }
  unsigned modifiers_size() const { return modifiers_Size; }
  llvm::iterator_range<modifiers_iterator> modifiers() const { return llvm::make_range(modifiers_begin(), modifiers_end()); }


  typedef Expr ** steps_iterator;
  steps_iterator steps_begin() const { return steps_; }
  steps_iterator steps_end() const { return steps_ + steps_Size; }
  unsigned steps_size() const { return steps_Size; }
  llvm::iterator_range<steps_iterator> steps() const { return llvm::make_range(steps_begin(), steps_end()); }



    void printPrettyPragma(raw_ostream & OS, const PrintingPolicy &Policy)
        const {
      if (getBranchState() != BS_Undefined)
        OS << ' ' << ConvertBranchStateTyToStr(getBranchState());
      if (auto *E = getSimdlen()) {
        OS << " simdlen(";
        E->printPretty(OS, nullptr, Policy);
        OS << ")";
      }
      if (uniforms_size() > 0) {
        OS << " uniform";
        StringRef Sep = "(";
        for (auto *E : uniforms()) {
          OS << Sep;
          E->printPretty(OS, nullptr, Policy);
          Sep = ", ";
        }
        OS << ")";
      }
      alignments_iterator NI = alignments_begin();
      for (auto *E : aligneds()) {
        OS << " aligned(";
        E->printPretty(OS, nullptr, Policy);
        if (*NI) {
          OS << ": ";
          (*NI)->printPretty(OS, nullptr, Policy);
        }
        OS << ")";
        ++NI;
      }
      steps_iterator I = steps_begin();
      modifiers_iterator MI = modifiers_begin();
      for (auto *E : linears()) {
        OS << " linear(";
        if (*MI != OMPC_LINEAR_unknown)
          OS << getOpenMPSimpleClauseTypeName(OMPC_linear, *MI) << "(";
        E->printPretty(OS, nullptr, Policy);
        if (*MI != OMPC_LINEAR_unknown)
          OS << ")";
        if (*I) {
          OS << ": ";
          (*I)->printPretty(OS, nullptr, Policy);
        }
        OS << ")";
        ++I;
        ++MI;
      }
    }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::OMPDeclareSimdDecl; }
};

class OMPDeclareTargetDeclAttr : public InheritableAttr {
public:
  enum MapTypeTy {
    MT_To,
    MT_Link
  };
private:
  MapTypeTy mapType;

public:
  enum DevTypeTy {
    DT_Host,
    DT_NoHost,
    DT_Any
  };
private:
  DevTypeTy devType;

public:
  static OMPDeclareTargetDeclAttr *CreateImplicit(ASTContext &Ctx, MapTypeTy MapType, DevTypeTy DevType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPDeclareTargetDeclAttr(Ctx, CommonInfo, MapType, DevType);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPDeclareTargetDeclAttr *Create(ASTContext &Ctx, MapTypeTy MapType, DevTypeTy DevType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPDeclareTargetDeclAttr(Ctx, CommonInfo, MapType, DevType);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPDeclareTargetDeclAttr *CreateImplicit(ASTContext &Ctx, MapTypeTy MapType, DevTypeTy DevType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, MapType, DevType, I);
  }
  static OMPDeclareTargetDeclAttr *Create(ASTContext &Ctx, MapTypeTy MapType, DevTypeTy DevType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, MapType, DevType, I);
  }
  OMPDeclareTargetDeclAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , MapTypeTy MapType
              , DevTypeTy DevType
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OMPDeclareTargetDecl, false, false)
              , mapType(MapType)
              , devType(DevType)
  {
  }

  OMPDeclareTargetDeclAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  MapTypeTy getMapType() const {
    return mapType;
  }

  static bool ConvertStrToMapTypeTy(StringRef Val, MapTypeTy &Out) {
    Optional<MapTypeTy> R = llvm::StringSwitch<Optional<MapTypeTy>>(Val)
      .Case("to", OMPDeclareTargetDeclAttr::MT_To)
      .Case("link", OMPDeclareTargetDeclAttr::MT_Link)
      .Default(Optional<MapTypeTy>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertMapTypeTyToStr(MapTypeTy Val) {
    switch(Val) {
    case OMPDeclareTargetDeclAttr::MT_To: return "to";
    case OMPDeclareTargetDeclAttr::MT_Link: return "link";
    }
    llvm_unreachable("No enumerator with that value");
  }
  DevTypeTy getDevType() const {
    return devType;
  }

  static bool ConvertStrToDevTypeTy(StringRef Val, DevTypeTy &Out) {
    Optional<DevTypeTy> R = llvm::StringSwitch<Optional<DevTypeTy>>(Val)
      .Case("host", OMPDeclareTargetDeclAttr::DT_Host)
      .Case("nohost", OMPDeclareTargetDeclAttr::DT_NoHost)
      .Case("any", OMPDeclareTargetDeclAttr::DT_Any)
      .Default(Optional<DevTypeTy>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertDevTypeTyToStr(DevTypeTy Val) {
    switch(Val) {
    case OMPDeclareTargetDeclAttr::DT_Host: return "host";
    case OMPDeclareTargetDeclAttr::DT_NoHost: return "nohost";
    case OMPDeclareTargetDeclAttr::DT_Any: return "any";
    }
    llvm_unreachable("No enumerator with that value");
  }

    void printPrettyPragma(raw_ostream &OS, const PrintingPolicy &Policy) const {
      // Use fake syntax because it is for testing and debugging purpose only.
      if (getDevType() != DT_Any)
        OS << " device_type(" << ConvertDevTypeTyToStr(getDevType()) << ")";
      if (getMapType() != MT_To)
        OS << ' ' << ConvertMapTypeTyToStr(getMapType());
    }
    static llvm::Optional<MapTypeTy>
    isDeclareTargetDeclaration(const ValueDecl *VD) {
      if (!VD->hasAttrs())
        return llvm::None;
      if (const auto *Attr = VD->getAttr<OMPDeclareTargetDeclAttr>())
        return Attr->getMapType();

      return llvm::None;
    }
    static llvm::Optional<DevTypeTy> getDeviceType(const ValueDecl *VD) {
      if (!VD->hasAttrs())
        return llvm::None;
      if (const auto *Attr = VD->getAttr<OMPDeclareTargetDeclAttr>())
        return Attr->getDevType();

      return llvm::None;
    }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::OMPDeclareTargetDecl; }
};

class OMPDeclareVariantAttr : public InheritableAttr {
Expr * variantFuncRef;

  unsigned scores_Size;
  Expr * *scores_;

  unsigned ctxSelectorSets_Size;
  unsigned *ctxSelectorSets_;

  unsigned ctxSelectors_Size;
  unsigned *ctxSelectors_;

  unsigned implVendors_Size;
  StringRef *implVendors_;

  unsigned deviceKinds_Size;
  StringRef *deviceKinds_;

public:
  static OMPDeclareVariantAttr *CreateImplicit(ASTContext &Ctx, Expr * VariantFuncRef, Expr * *Scores, unsigned ScoresSize, unsigned *CtxSelectorSets, unsigned CtxSelectorSetsSize, unsigned *CtxSelectors, unsigned CtxSelectorsSize, StringRef *ImplVendors, unsigned ImplVendorsSize, StringRef *DeviceKinds, unsigned DeviceKindsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPDeclareVariantAttr(Ctx, CommonInfo, VariantFuncRef, Scores, ScoresSize, CtxSelectorSets, CtxSelectorSetsSize, CtxSelectors, CtxSelectorsSize, ImplVendors, ImplVendorsSize, DeviceKinds, DeviceKindsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPDeclareVariantAttr *Create(ASTContext &Ctx, Expr * VariantFuncRef, Expr * *Scores, unsigned ScoresSize, unsigned *CtxSelectorSets, unsigned CtxSelectorSetsSize, unsigned *CtxSelectors, unsigned CtxSelectorsSize, StringRef *ImplVendors, unsigned ImplVendorsSize, StringRef *DeviceKinds, unsigned DeviceKindsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPDeclareVariantAttr(Ctx, CommonInfo, VariantFuncRef, Scores, ScoresSize, CtxSelectorSets, CtxSelectorSetsSize, CtxSelectors, CtxSelectorsSize, ImplVendors, ImplVendorsSize, DeviceKinds, DeviceKindsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPDeclareVariantAttr *CreateImplicit(ASTContext &Ctx, Expr * VariantFuncRef, Expr * *Scores, unsigned ScoresSize, unsigned *CtxSelectorSets, unsigned CtxSelectorSetsSize, unsigned *CtxSelectors, unsigned CtxSelectorsSize, StringRef *ImplVendors, unsigned ImplVendorsSize, StringRef *DeviceKinds, unsigned DeviceKindsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, VariantFuncRef, Scores, ScoresSize, CtxSelectorSets, CtxSelectorSetsSize, CtxSelectors, CtxSelectorsSize, ImplVendors, ImplVendorsSize, DeviceKinds, DeviceKindsSize, I);
  }
  static OMPDeclareVariantAttr *Create(ASTContext &Ctx, Expr * VariantFuncRef, Expr * *Scores, unsigned ScoresSize, unsigned *CtxSelectorSets, unsigned CtxSelectorSetsSize, unsigned *CtxSelectors, unsigned CtxSelectorsSize, StringRef *ImplVendors, unsigned ImplVendorsSize, StringRef *DeviceKinds, unsigned DeviceKindsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, VariantFuncRef, Scores, ScoresSize, CtxSelectorSets, CtxSelectorSetsSize, CtxSelectors, CtxSelectorsSize, ImplVendors, ImplVendorsSize, DeviceKinds, DeviceKindsSize, I);
  }
  OMPDeclareVariantAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * VariantFuncRef
              , Expr * *Scores, unsigned ScoresSize
              , unsigned *CtxSelectorSets, unsigned CtxSelectorSetsSize
              , unsigned *CtxSelectors, unsigned CtxSelectorsSize
              , StringRef *ImplVendors, unsigned ImplVendorsSize
              , StringRef *DeviceKinds, unsigned DeviceKindsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OMPDeclareVariant, false, true)
              , variantFuncRef(VariantFuncRef)
              , scores_Size(ScoresSize), scores_(new (Ctx, 16) Expr *[scores_Size])
              , ctxSelectorSets_Size(CtxSelectorSetsSize), ctxSelectorSets_(new (Ctx, 16) unsigned[ctxSelectorSets_Size])
              , ctxSelectors_Size(CtxSelectorsSize), ctxSelectors_(new (Ctx, 16) unsigned[ctxSelectors_Size])
              , implVendors_Size(ImplVendorsSize), implVendors_(new (Ctx, 16) StringRef[implVendors_Size])
              , deviceKinds_Size(DeviceKindsSize), deviceKinds_(new (Ctx, 16) StringRef[deviceKinds_Size])
  {
    std::copy(Scores, Scores + scores_Size, scores_);
    std::copy(CtxSelectorSets, CtxSelectorSets + ctxSelectorSets_Size, ctxSelectorSets_);
    std::copy(CtxSelectors, CtxSelectors + ctxSelectors_Size, ctxSelectors_);
    for (size_t I = 0, E = implVendors_Size; I != E;
         ++I) {
      StringRef Ref = ImplVendors[I];
      if (!Ref.empty()) {
        char *Mem = new (Ctx, 1) char[Ref.size()];
        std::memcpy(Mem, Ref.data(), Ref.size());
        implVendors_[I] = StringRef(Mem, Ref.size());
      }
    }
    for (size_t I = 0, E = deviceKinds_Size; I != E;
         ++I) {
      StringRef Ref = DeviceKinds[I];
      if (!Ref.empty()) {
        char *Mem = new (Ctx, 1) char[Ref.size()];
        std::memcpy(Mem, Ref.data(), Ref.size());
        deviceKinds_[I] = StringRef(Mem, Ref.size());
      }
    }
  }

  OMPDeclareVariantAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * VariantFuncRef
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OMPDeclareVariant, false, true)
              , variantFuncRef(VariantFuncRef)
              , scores_Size(0), scores_(nullptr)
              , ctxSelectorSets_Size(0), ctxSelectorSets_(nullptr)
              , ctxSelectors_Size(0), ctxSelectors_(nullptr)
              , implVendors_Size(0), implVendors_(nullptr)
              , deviceKinds_Size(0), deviceKinds_(nullptr)
  {
  }

  OMPDeclareVariantAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getVariantFuncRef() const {
    return variantFuncRef;
  }

  typedef Expr ** scores_iterator;
  scores_iterator scores_begin() const { return scores_; }
  scores_iterator scores_end() const { return scores_ + scores_Size; }
  unsigned scores_size() const { return scores_Size; }
  llvm::iterator_range<scores_iterator> scores() const { return llvm::make_range(scores_begin(), scores_end()); }


  typedef unsigned* ctxSelectorSets_iterator;
  ctxSelectorSets_iterator ctxSelectorSets_begin() const { return ctxSelectorSets_; }
  ctxSelectorSets_iterator ctxSelectorSets_end() const { return ctxSelectorSets_ + ctxSelectorSets_Size; }
  unsigned ctxSelectorSets_size() const { return ctxSelectorSets_Size; }
  llvm::iterator_range<ctxSelectorSets_iterator> ctxSelectorSets() const { return llvm::make_range(ctxSelectorSets_begin(), ctxSelectorSets_end()); }


  typedef unsigned* ctxSelectors_iterator;
  ctxSelectors_iterator ctxSelectors_begin() const { return ctxSelectors_; }
  ctxSelectors_iterator ctxSelectors_end() const { return ctxSelectors_ + ctxSelectors_Size; }
  unsigned ctxSelectors_size() const { return ctxSelectors_Size; }
  llvm::iterator_range<ctxSelectors_iterator> ctxSelectors() const { return llvm::make_range(ctxSelectors_begin(), ctxSelectors_end()); }


  typedef StringRef* implVendors_iterator;
  implVendors_iterator implVendors_begin() const { return implVendors_; }
  implVendors_iterator implVendors_end() const { return implVendors_ + implVendors_Size; }
  unsigned implVendors_size() const { return implVendors_Size; }
  llvm::iterator_range<implVendors_iterator> implVendors() const { return llvm::make_range(implVendors_begin(), implVendors_end()); }


  typedef StringRef* deviceKinds_iterator;
  deviceKinds_iterator deviceKinds_begin() const { return deviceKinds_; }
  deviceKinds_iterator deviceKinds_end() const { return deviceKinds_ + deviceKinds_Size; }
  unsigned deviceKinds_size() const { return deviceKinds_Size; }
  llvm::iterator_range<deviceKinds_iterator> deviceKinds() const { return llvm::make_range(deviceKinds_begin(), deviceKinds_end()); }



    void printScore(raw_ostream & OS, const PrintingPolicy &Policy, unsigned I) const {
      if (const Expr *E = *std::next(scores_begin(), I)) {
        OS << "score(";
        E->printPretty(OS, nullptr, Policy);
        OS << "):";
      }
    }
    void printPrettyPragma(raw_ostream & OS, const PrintingPolicy &Policy)
        const {
      if (const Expr *E = getVariantFuncRef()) {
        OS << "(";
        E->printPretty(OS, nullptr, Policy);
        OS << ")";
      }
      // TODO: add printing of real context selectors.
      OS << " match(";
      int Used[OMP_CTX_SET_unknown] = {0};
      for (unsigned I = 0, E = ctxSelectorSets_size(); I < E; ++I) {
        auto CtxSet = static_cast<OpenMPContextSelectorSetKind>(
            *std::next(ctxSelectorSets_begin(), I));
        if (Used[CtxSet])
          continue;
        if (I > 0)
          OS << ",";
        switch (CtxSet) {
        case OMP_CTX_SET_implementation:
          OS << "implementation={";
          break;
        case OMP_CTX_SET_device:
          OS << "device={";
          break;
        case OMP_CTX_SET_unknown:
          llvm_unreachable("Unknown context selector set.");
        }
        Used[CtxSet] = 1;
        for (unsigned K = I, EK = ctxSelectors_size(); K < EK; ++K) {
          auto CtxSetK = static_cast<OpenMPContextSelectorSetKind>(
              *std::next(ctxSelectorSets_begin(), K));
          if (CtxSet != CtxSetK)
            continue;
          if (K != I)
            OS << ",";
          auto Ctx = static_cast<OpenMPContextSelectorKind>(
              *std::next(ctxSelectors_begin(), K));
          switch (Ctx) {
          case OMP_CTX_vendor:
            assert(CtxSet == OMP_CTX_SET_implementation &&
                   "Expected implementation context selector set.");
            OS << "vendor(";
            printScore(OS, Policy, K);
            if (implVendors_size() > 0) {
              OS << *implVendors(). begin();
              for (StringRef VendorName : llvm::drop_begin(implVendors(), 1))
                OS << ", " << VendorName;
            }
            OS << ")";
            break;
          case OMP_CTX_kind:
            assert(CtxSet == OMP_CTX_SET_device &&
                   "Expected device context selector set.");
            OS << "kind(";
            if (deviceKinds_size() > 0) {
              OS << *deviceKinds().begin();
              for (StringRef KindName : llvm::drop_begin(deviceKinds(), 1))
                OS << ", " << KindName;
            }
            OS << ")";
            break;
          case OMP_CTX_unknown:
            llvm_unreachable("Unknown context selector.");
          }
        }
        OS << "}";
      }
      OS << ")";
    }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::OMPDeclareVariant; }
};

class OMPReferencedVarAttr : public Attr {
Expr * ref;

public:
  static OMPReferencedVarAttr *CreateImplicit(ASTContext &Ctx, Expr * Ref, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPReferencedVarAttr(Ctx, CommonInfo, Ref);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPReferencedVarAttr *Create(ASTContext &Ctx, Expr * Ref, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPReferencedVarAttr(Ctx, CommonInfo, Ref);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPReferencedVarAttr *CreateImplicit(ASTContext &Ctx, Expr * Ref, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Ref, I);
  }
  static OMPReferencedVarAttr *Create(ASTContext &Ctx, Expr * Ref, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Ref, I);
  }
  OMPReferencedVarAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Ref
             )
    : Attr(Ctx, CommonInfo, attr::OMPReferencedVar, false)
              , ref(Ref)
  {
  }

  OMPReferencedVarAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getRef() const {
    return ref;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::OMPReferencedVar; }
};

class OMPThreadPrivateDeclAttr : public InheritableAttr {
public:
  static OMPThreadPrivateDeclAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPThreadPrivateDeclAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPThreadPrivateDeclAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OMPThreadPrivateDeclAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OMPThreadPrivateDeclAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OMPThreadPrivateDeclAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OMPThreadPrivateDeclAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OMPThreadPrivateDecl, false, false)
  {
  }

  OMPThreadPrivateDeclAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OMPThreadPrivateDecl; }
};

class OSConsumedAttr : public InheritableParamAttr {
public:
  static OSConsumedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSConsumedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSConsumedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSConsumedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSConsumedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OSConsumedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OSConsumedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::OSConsumed, false, false)
  {
  }

  OSConsumedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OSConsumed; }
};

class OSConsumesThisAttr : public InheritableAttr {
public:
  static OSConsumesThisAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSConsumesThisAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSConsumesThisAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSConsumesThisAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSConsumesThisAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OSConsumesThisAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OSConsumesThisAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OSConsumesThis, false, false)
  {
  }

  OSConsumesThisAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OSConsumesThis; }
};

class OSReturnsNotRetainedAttr : public InheritableAttr {
public:
  static OSReturnsNotRetainedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSReturnsNotRetainedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSReturnsNotRetainedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSReturnsNotRetainedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSReturnsNotRetainedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OSReturnsNotRetainedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OSReturnsNotRetainedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OSReturnsNotRetained, false, false)
  {
  }

  OSReturnsNotRetainedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OSReturnsNotRetained; }
};

class OSReturnsRetainedAttr : public InheritableAttr {
public:
  static OSReturnsRetainedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSReturnsRetainedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSReturnsRetainedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSReturnsRetainedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSReturnsRetainedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OSReturnsRetainedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OSReturnsRetainedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OSReturnsRetained, false, false)
  {
  }

  OSReturnsRetainedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OSReturnsRetained; }
};

class OSReturnsRetainedOnNonZeroAttr : public InheritableAttr {
public:
  static OSReturnsRetainedOnNonZeroAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSReturnsRetainedOnNonZeroAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSReturnsRetainedOnNonZeroAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSReturnsRetainedOnNonZeroAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSReturnsRetainedOnNonZeroAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OSReturnsRetainedOnNonZeroAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OSReturnsRetainedOnNonZeroAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OSReturnsRetainedOnNonZero, false, false)
  {
  }

  OSReturnsRetainedOnNonZeroAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OSReturnsRetainedOnNonZero; }
};

class OSReturnsRetainedOnZeroAttr : public InheritableAttr {
public:
  static OSReturnsRetainedOnZeroAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSReturnsRetainedOnZeroAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSReturnsRetainedOnZeroAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OSReturnsRetainedOnZeroAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OSReturnsRetainedOnZeroAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OSReturnsRetainedOnZeroAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OSReturnsRetainedOnZeroAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OSReturnsRetainedOnZero, false, false)
  {
  }

  OSReturnsRetainedOnZeroAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OSReturnsRetainedOnZero; }
};

class ObjCBoxableAttr : public Attr {
public:
  static ObjCBoxableAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCBoxableAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCBoxableAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCBoxableAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCBoxableAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCBoxableAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCBoxableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::ObjCBoxable, false)
  {
  }

  ObjCBoxableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCBoxable; }
};

class ObjCBridgeAttr : public InheritableAttr {
IdentifierInfo * bridgedType;

public:
  static ObjCBridgeAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * BridgedType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCBridgeAttr(Ctx, CommonInfo, BridgedType);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCBridgeAttr *Create(ASTContext &Ctx, IdentifierInfo * BridgedType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCBridgeAttr(Ctx, CommonInfo, BridgedType);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCBridgeAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * BridgedType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, BridgedType, I);
  }
  static ObjCBridgeAttr *Create(ASTContext &Ctx, IdentifierInfo * BridgedType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, BridgedType, I);
  }
  ObjCBridgeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * BridgedType
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCBridge, false, false)
              , bridgedType(BridgedType)
  {
  }

  ObjCBridgeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getBridgedType() const {
    return bridgedType;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCBridge; }
};

class ObjCBridgeMutableAttr : public InheritableAttr {
IdentifierInfo * bridgedType;

public:
  static ObjCBridgeMutableAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * BridgedType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCBridgeMutableAttr(Ctx, CommonInfo, BridgedType);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCBridgeMutableAttr *Create(ASTContext &Ctx, IdentifierInfo * BridgedType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCBridgeMutableAttr(Ctx, CommonInfo, BridgedType);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCBridgeMutableAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * BridgedType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, BridgedType, I);
  }
  static ObjCBridgeMutableAttr *Create(ASTContext &Ctx, IdentifierInfo * BridgedType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, BridgedType, I);
  }
  ObjCBridgeMutableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * BridgedType
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCBridgeMutable, false, false)
              , bridgedType(BridgedType)
  {
  }

  ObjCBridgeMutableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getBridgedType() const {
    return bridgedType;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCBridgeMutable; }
};

class ObjCBridgeRelatedAttr : public InheritableAttr {
IdentifierInfo * relatedClass;

IdentifierInfo * classMethod;

IdentifierInfo * instanceMethod;

public:
  static ObjCBridgeRelatedAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * RelatedClass, IdentifierInfo * ClassMethod, IdentifierInfo * InstanceMethod, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCBridgeRelatedAttr(Ctx, CommonInfo, RelatedClass, ClassMethod, InstanceMethod);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCBridgeRelatedAttr *Create(ASTContext &Ctx, IdentifierInfo * RelatedClass, IdentifierInfo * ClassMethod, IdentifierInfo * InstanceMethod, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCBridgeRelatedAttr(Ctx, CommonInfo, RelatedClass, ClassMethod, InstanceMethod);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCBridgeRelatedAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * RelatedClass, IdentifierInfo * ClassMethod, IdentifierInfo * InstanceMethod, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, RelatedClass, ClassMethod, InstanceMethod, I);
  }
  static ObjCBridgeRelatedAttr *Create(ASTContext &Ctx, IdentifierInfo * RelatedClass, IdentifierInfo * ClassMethod, IdentifierInfo * InstanceMethod, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, RelatedClass, ClassMethod, InstanceMethod, I);
  }
  ObjCBridgeRelatedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * RelatedClass
              , IdentifierInfo * ClassMethod
              , IdentifierInfo * InstanceMethod
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCBridgeRelated, false, false)
              , relatedClass(RelatedClass)
              , classMethod(ClassMethod)
              , instanceMethod(InstanceMethod)
  {
  }

  ObjCBridgeRelatedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getRelatedClass() const {
    return relatedClass;
  }

  IdentifierInfo * getClassMethod() const {
    return classMethod;
  }

  IdentifierInfo * getInstanceMethod() const {
    return instanceMethod;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCBridgeRelated; }
};

class ObjCClassStubAttr : public Attr {
public:
  static ObjCClassStubAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCClassStubAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCClassStubAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCClassStubAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCClassStubAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCClassStubAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCClassStubAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::ObjCClassStub, false)
  {
  }

  ObjCClassStubAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCClassStub; }
};

class ObjCDesignatedInitializerAttr : public Attr {
public:
  static ObjCDesignatedInitializerAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCDesignatedInitializerAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCDesignatedInitializerAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCDesignatedInitializerAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCDesignatedInitializerAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCDesignatedInitializerAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCDesignatedInitializerAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::ObjCDesignatedInitializer, false)
  {
  }

  ObjCDesignatedInitializerAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCDesignatedInitializer; }
};

class ObjCDirectAttr : public Attr {
public:
  static ObjCDirectAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCDirectAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCDirectAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCDirectAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCDirectAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCDirectAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCDirectAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::ObjCDirect, false)
  {
  }

  ObjCDirectAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCDirect; }
};

class ObjCDirectMembersAttr : public Attr {
public:
  static ObjCDirectMembersAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCDirectMembersAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCDirectMembersAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCDirectMembersAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCDirectMembersAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCDirectMembersAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCDirectMembersAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::ObjCDirectMembers, false)
  {
  }

  ObjCDirectMembersAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCDirectMembers; }
};

class ObjCExceptionAttr : public InheritableAttr {
public:
  static ObjCExceptionAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCExceptionAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCExceptionAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCExceptionAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCExceptionAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCExceptionAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCExceptionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCException, false, false)
  {
  }

  ObjCExceptionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCException; }
};

class ObjCExplicitProtocolImplAttr : public InheritableAttr {
public:
  static ObjCExplicitProtocolImplAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCExplicitProtocolImplAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCExplicitProtocolImplAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCExplicitProtocolImplAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCExplicitProtocolImplAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCExplicitProtocolImplAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCExplicitProtocolImplAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCExplicitProtocolImpl, false, false)
  {
  }

  ObjCExplicitProtocolImplAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCExplicitProtocolImpl; }
};

class ObjCExternallyRetainedAttr : public InheritableAttr {
public:
  static ObjCExternallyRetainedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCExternallyRetainedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCExternallyRetainedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCExternallyRetainedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCExternallyRetainedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCExternallyRetainedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCExternallyRetainedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCExternallyRetained, false, false)
  {
  }

  ObjCExternallyRetainedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCExternallyRetained; }
};

class ObjCGCAttr : public TypeAttr {
IdentifierInfo * kind;

public:
  static ObjCGCAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Kind, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCGCAttr(Ctx, CommonInfo, Kind);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCGCAttr *Create(ASTContext &Ctx, IdentifierInfo * Kind, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCGCAttr(Ctx, CommonInfo, Kind);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCGCAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Kind, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Kind, I);
  }
  static ObjCGCAttr *Create(ASTContext &Ctx, IdentifierInfo * Kind, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Kind, I);
  }
  ObjCGCAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * Kind
             )
    : TypeAttr(Ctx, CommonInfo, attr::ObjCGC, false)
              , kind(Kind)
  {
  }

  ObjCGCAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getKind() const {
    return kind;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCGC; }
};

class ObjCIndependentClassAttr : public InheritableAttr {
public:
  static ObjCIndependentClassAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCIndependentClassAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCIndependentClassAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCIndependentClassAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCIndependentClassAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCIndependentClassAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCIndependentClassAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCIndependentClass, false, false)
  {
  }

  ObjCIndependentClassAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCIndependentClass; }
};

class ObjCInertUnsafeUnretainedAttr : public TypeAttr {
public:
  static ObjCInertUnsafeUnretainedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCInertUnsafeUnretainedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCInertUnsafeUnretainedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCInertUnsafeUnretainedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCInertUnsafeUnretainedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCInertUnsafeUnretainedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCInertUnsafeUnretainedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::ObjCInertUnsafeUnretained, false)
  {
  }

  ObjCInertUnsafeUnretainedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCInertUnsafeUnretained; }
};

class ObjCKindOfAttr : public TypeAttr {
public:
  static ObjCKindOfAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCKindOfAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCKindOfAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCKindOfAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCKindOfAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCKindOfAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCKindOfAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::ObjCKindOf, false)
  {
  }

  ObjCKindOfAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCKindOf; }
};

class ObjCMethodFamilyAttr : public InheritableAttr {
public:
  enum FamilyKind {
    OMF_None,
    OMF_alloc,
    OMF_copy,
    OMF_init,
    OMF_mutableCopy,
    OMF_new
  };
private:
  FamilyKind family;

public:
  static ObjCMethodFamilyAttr *CreateImplicit(ASTContext &Ctx, FamilyKind Family, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCMethodFamilyAttr(Ctx, CommonInfo, Family);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCMethodFamilyAttr *Create(ASTContext &Ctx, FamilyKind Family, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCMethodFamilyAttr(Ctx, CommonInfo, Family);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCMethodFamilyAttr *CreateImplicit(ASTContext &Ctx, FamilyKind Family, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Family, I);
  }
  static ObjCMethodFamilyAttr *Create(ASTContext &Ctx, FamilyKind Family, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Family, I);
  }
  ObjCMethodFamilyAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , FamilyKind Family
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCMethodFamily, false, false)
              , family(Family)
  {
  }

  ObjCMethodFamilyAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  FamilyKind getFamily() const {
    return family;
  }

  static bool ConvertStrToFamilyKind(StringRef Val, FamilyKind &Out) {
    Optional<FamilyKind> R = llvm::StringSwitch<Optional<FamilyKind>>(Val)
      .Case("none", ObjCMethodFamilyAttr::OMF_None)
      .Case("alloc", ObjCMethodFamilyAttr::OMF_alloc)
      .Case("copy", ObjCMethodFamilyAttr::OMF_copy)
      .Case("init", ObjCMethodFamilyAttr::OMF_init)
      .Case("mutableCopy", ObjCMethodFamilyAttr::OMF_mutableCopy)
      .Case("new", ObjCMethodFamilyAttr::OMF_new)
      .Default(Optional<FamilyKind>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertFamilyKindToStr(FamilyKind Val) {
    switch(Val) {
    case ObjCMethodFamilyAttr::OMF_None: return "none";
    case ObjCMethodFamilyAttr::OMF_alloc: return "alloc";
    case ObjCMethodFamilyAttr::OMF_copy: return "copy";
    case ObjCMethodFamilyAttr::OMF_init: return "init";
    case ObjCMethodFamilyAttr::OMF_mutableCopy: return "mutableCopy";
    case ObjCMethodFamilyAttr::OMF_new: return "new";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCMethodFamily; }
};

class ObjCNSObjectAttr : public InheritableAttr {
public:
  static ObjCNSObjectAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCNSObjectAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCNSObjectAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCNSObjectAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCNSObjectAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCNSObjectAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCNSObjectAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCNSObject, false, false)
  {
  }

  ObjCNSObjectAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCNSObject; }
};

class ObjCNonLazyClassAttr : public Attr {
public:
  static ObjCNonLazyClassAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCNonLazyClassAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCNonLazyClassAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCNonLazyClassAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCNonLazyClassAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCNonLazyClassAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCNonLazyClassAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::ObjCNonLazyClass, false)
  {
  }

  ObjCNonLazyClassAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCNonLazyClass; }
};

class ObjCOwnershipAttr : public InheritableAttr {
IdentifierInfo * kind;

public:
  static ObjCOwnershipAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Kind, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCOwnershipAttr(Ctx, CommonInfo, Kind);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCOwnershipAttr *Create(ASTContext &Ctx, IdentifierInfo * Kind, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCOwnershipAttr(Ctx, CommonInfo, Kind);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCOwnershipAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Kind, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Kind, I);
  }
  static ObjCOwnershipAttr *Create(ASTContext &Ctx, IdentifierInfo * Kind, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Kind, I);
  }
  ObjCOwnershipAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * Kind
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCOwnership, false, false)
              , kind(Kind)
  {
  }

  ObjCOwnershipAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getKind() const {
    return kind;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCOwnership; }
};

class ObjCPreciseLifetimeAttr : public InheritableAttr {
public:
  static ObjCPreciseLifetimeAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCPreciseLifetimeAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCPreciseLifetimeAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCPreciseLifetimeAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCPreciseLifetimeAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCPreciseLifetimeAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCPreciseLifetimeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCPreciseLifetime, false, false)
  {
  }

  ObjCPreciseLifetimeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCPreciseLifetime; }
};

class ObjCRequiresPropertyDefsAttr : public InheritableAttr {
public:
  static ObjCRequiresPropertyDefsAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRequiresPropertyDefsAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRequiresPropertyDefsAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRequiresPropertyDefsAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRequiresPropertyDefsAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCRequiresPropertyDefsAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCRequiresPropertyDefsAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCRequiresPropertyDefs, false, false)
  {
  }

  ObjCRequiresPropertyDefsAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCRequiresPropertyDefs; }
};

class ObjCRequiresSuperAttr : public InheritableAttr {
public:
  static ObjCRequiresSuperAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRequiresSuperAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRequiresSuperAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRequiresSuperAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRequiresSuperAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCRequiresSuperAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCRequiresSuperAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCRequiresSuper, false, false)
  {
  }

  ObjCRequiresSuperAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCRequiresSuper; }
};

class ObjCReturnsInnerPointerAttr : public InheritableAttr {
public:
  static ObjCReturnsInnerPointerAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCReturnsInnerPointerAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCReturnsInnerPointerAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCReturnsInnerPointerAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCReturnsInnerPointerAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCReturnsInnerPointerAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCReturnsInnerPointerAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCReturnsInnerPointer, false, false)
  {
  }

  ObjCReturnsInnerPointerAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCReturnsInnerPointer; }
};

class ObjCRootClassAttr : public InheritableAttr {
public:
  static ObjCRootClassAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRootClassAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRootClassAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRootClassAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRootClassAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCRootClassAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCRootClassAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCRootClass, false, false)
  {
  }

  ObjCRootClassAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCRootClass; }
};

class ObjCRuntimeNameAttr : public Attr {
unsigned metadataNameLength;
char *metadataName;

public:
  static ObjCRuntimeNameAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef MetadataName, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRuntimeNameAttr(Ctx, CommonInfo, MetadataName);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRuntimeNameAttr *Create(ASTContext &Ctx, llvm::StringRef MetadataName, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRuntimeNameAttr(Ctx, CommonInfo, MetadataName);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRuntimeNameAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef MetadataName, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, MetadataName, I);
  }
  static ObjCRuntimeNameAttr *Create(ASTContext &Ctx, llvm::StringRef MetadataName, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, MetadataName, I);
  }
  ObjCRuntimeNameAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef MetadataName
             )
    : Attr(Ctx, CommonInfo, attr::ObjCRuntimeName, false)
              , metadataNameLength(MetadataName.size()),metadataName(new (Ctx, 1) char[metadataNameLength])
  {
      if (!MetadataName.empty())
        std::memcpy(metadataName, MetadataName.data(), metadataNameLength);
  }

  ObjCRuntimeNameAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getMetadataName() const {
    return llvm::StringRef(metadataName, metadataNameLength);
  }
  unsigned getMetadataNameLength() const {
    return metadataNameLength;
  }
  void setMetadataName(ASTContext &C, llvm::StringRef S) {
    metadataNameLength = S.size();
    this->metadataName = new (C, 1) char [metadataNameLength];
    if (!S.empty())
      std::memcpy(this->metadataName, S.data(), metadataNameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCRuntimeName; }
};

class ObjCRuntimeVisibleAttr : public Attr {
public:
  static ObjCRuntimeVisibleAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRuntimeVisibleAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRuntimeVisibleAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCRuntimeVisibleAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCRuntimeVisibleAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCRuntimeVisibleAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCRuntimeVisibleAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::ObjCRuntimeVisible, false)
  {
  }

  ObjCRuntimeVisibleAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCRuntimeVisible; }
};

class ObjCSubclassingRestrictedAttr : public InheritableAttr {
public:
  static ObjCSubclassingRestrictedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCSubclassingRestrictedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCSubclassingRestrictedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ObjCSubclassingRestrictedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ObjCSubclassingRestrictedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ObjCSubclassingRestrictedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ObjCSubclassingRestrictedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ObjCSubclassingRestricted, false, false)
  {
  }

  ObjCSubclassingRestrictedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ObjCSubclassingRestricted; }
};

class OpenCLAccessAttr : public Attr {
public:
  enum Spelling {
    Keyword_read_only = 0,
    Keyword_write_only = 2,
    Keyword_read_write = 4,
  SpellingNotCalculated = 15

  };

  static OpenCLAccessAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLAccessAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLAccessAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLAccessAttr(Ctx, CommonInfo);
    return A;
  }

  static OpenCLAccessAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLAccessAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static OpenCLAccessAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLAccessAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  OpenCLAccessAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::OpenCLAccess, false)
  {
  }

  OpenCLAccessAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Keyword_read_only;
    case 1: return Keyword_read_only;
    case 2: return Keyword_write_only;
    case 3: return Keyword_write_only;
    case 4: return Keyword_read_write;
    case 5: return Keyword_read_write;
  }
  }
  bool isReadOnly() const { return getAttributeSpellingListIndex() == 0 ||
    getAttributeSpellingListIndex() == 1; }
  bool isReadWrite() const { return getAttributeSpellingListIndex() == 4 ||
    getAttributeSpellingListIndex() == 5; }
  bool isWriteOnly() const { return getAttributeSpellingListIndex() == 2 ||
    getAttributeSpellingListIndex() == 3; }


  static bool classof(const Attr *A) { return A->getKind() == attr::OpenCLAccess; }
};

class OpenCLConstantAddressSpaceAttr : public TypeAttr {
public:
  enum Spelling {
    Keyword_constant = 0,
    GNU_opencl_constant = 2,
    CXX11_clang_opencl_constant = 3,
    C2x_clang_opencl_constant = 4,
  SpellingNotCalculated = 15

  };

  static OpenCLConstantAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLConstantAddressSpaceAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLConstantAddressSpaceAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLConstantAddressSpaceAttr(Ctx, CommonInfo);
    return A;
  }

  static OpenCLConstantAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLConstantAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static OpenCLConstantAddressSpaceAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLConstantAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  OpenCLConstantAddressSpaceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::OpenCLConstantAddressSpace, false)
  {
  }

  OpenCLConstantAddressSpaceAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Keyword_constant;
    case 1: return Keyword_constant;
    case 2: return GNU_opencl_constant;
    case 3: return CXX11_clang_opencl_constant;
    case 4: return C2x_clang_opencl_constant;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::OpenCLConstantAddressSpace; }
};

class OpenCLGenericAddressSpaceAttr : public TypeAttr {
public:
  enum Spelling {
    Keyword_generic = 0,
    GNU_opencl_generic = 2,
    CXX11_clang_opencl_generic = 3,
    C2x_clang_opencl_generic = 4,
  SpellingNotCalculated = 15

  };

  static OpenCLGenericAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLGenericAddressSpaceAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLGenericAddressSpaceAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLGenericAddressSpaceAttr(Ctx, CommonInfo);
    return A;
  }

  static OpenCLGenericAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLGenericAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static OpenCLGenericAddressSpaceAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLGenericAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  OpenCLGenericAddressSpaceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::OpenCLGenericAddressSpace, false)
  {
  }

  OpenCLGenericAddressSpaceAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Keyword_generic;
    case 1: return Keyword_generic;
    case 2: return GNU_opencl_generic;
    case 3: return CXX11_clang_opencl_generic;
    case 4: return C2x_clang_opencl_generic;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::OpenCLGenericAddressSpace; }
};

class OpenCLGlobalAddressSpaceAttr : public TypeAttr {
public:
  enum Spelling {
    Keyword_global = 0,
    GNU_opencl_global = 2,
    CXX11_clang_opencl_global = 3,
    C2x_clang_opencl_global = 4,
  SpellingNotCalculated = 15

  };

  static OpenCLGlobalAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLGlobalAddressSpaceAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLGlobalAddressSpaceAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLGlobalAddressSpaceAttr(Ctx, CommonInfo);
    return A;
  }

  static OpenCLGlobalAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLGlobalAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static OpenCLGlobalAddressSpaceAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLGlobalAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  OpenCLGlobalAddressSpaceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::OpenCLGlobalAddressSpace, false)
  {
  }

  OpenCLGlobalAddressSpaceAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Keyword_global;
    case 1: return Keyword_global;
    case 2: return GNU_opencl_global;
    case 3: return CXX11_clang_opencl_global;
    case 4: return C2x_clang_opencl_global;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::OpenCLGlobalAddressSpace; }
};

class OpenCLIntelReqdSubGroupSizeAttr : public InheritableAttr {
unsigned subGroupSize;

public:
  static OpenCLIntelReqdSubGroupSizeAttr *CreateImplicit(ASTContext &Ctx, unsigned SubGroupSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLIntelReqdSubGroupSizeAttr(Ctx, CommonInfo, SubGroupSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLIntelReqdSubGroupSizeAttr *Create(ASTContext &Ctx, unsigned SubGroupSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLIntelReqdSubGroupSizeAttr(Ctx, CommonInfo, SubGroupSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLIntelReqdSubGroupSizeAttr *CreateImplicit(ASTContext &Ctx, unsigned SubGroupSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, SubGroupSize, I);
  }
  static OpenCLIntelReqdSubGroupSizeAttr *Create(ASTContext &Ctx, unsigned SubGroupSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, SubGroupSize, I);
  }
  OpenCLIntelReqdSubGroupSizeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned SubGroupSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OpenCLIntelReqdSubGroupSize, false, false)
              , subGroupSize(SubGroupSize)
  {
  }

  OpenCLIntelReqdSubGroupSizeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getSubGroupSize() const {
    return subGroupSize;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::OpenCLIntelReqdSubGroupSize; }
};

class OpenCLKernelAttr : public InheritableAttr {
public:
  static OpenCLKernelAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLKernelAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLKernelAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLKernelAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLKernelAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OpenCLKernelAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OpenCLKernelAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OpenCLKernel, false, false)
  {
  }

  OpenCLKernelAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OpenCLKernel; }
};

class OpenCLLocalAddressSpaceAttr : public TypeAttr {
public:
  enum Spelling {
    Keyword_local = 0,
    GNU_opencl_local = 2,
    CXX11_clang_opencl_local = 3,
    C2x_clang_opencl_local = 4,
  SpellingNotCalculated = 15

  };

  static OpenCLLocalAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLLocalAddressSpaceAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLLocalAddressSpaceAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLLocalAddressSpaceAttr(Ctx, CommonInfo);
    return A;
  }

  static OpenCLLocalAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLLocalAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static OpenCLLocalAddressSpaceAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLLocalAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  OpenCLLocalAddressSpaceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::OpenCLLocalAddressSpace, false)
  {
  }

  OpenCLLocalAddressSpaceAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Keyword_local;
    case 1: return Keyword_local;
    case 2: return GNU_opencl_local;
    case 3: return CXX11_clang_opencl_local;
    case 4: return C2x_clang_opencl_local;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::OpenCLLocalAddressSpace; }
};

class OpenCLPrivateAddressSpaceAttr : public TypeAttr {
public:
  enum Spelling {
    Keyword_private = 0,
    GNU_opencl_private = 2,
    CXX11_clang_opencl_private = 3,
    C2x_clang_opencl_private = 4,
  SpellingNotCalculated = 15

  };

  static OpenCLPrivateAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLPrivateAddressSpaceAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLPrivateAddressSpaceAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLPrivateAddressSpaceAttr(Ctx, CommonInfo);
    return A;
  }

  static OpenCLPrivateAddressSpaceAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLPrivateAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static OpenCLPrivateAddressSpaceAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OpenCLPrivateAddressSpaceAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  OpenCLPrivateAddressSpaceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::OpenCLPrivateAddressSpace, false)
  {
  }

  OpenCLPrivateAddressSpaceAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Keyword_private;
    case 1: return Keyword_private;
    case 2: return GNU_opencl_private;
    case 3: return CXX11_clang_opencl_private;
    case 4: return C2x_clang_opencl_private;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::OpenCLPrivateAddressSpace; }
};

class OpenCLUnrollHintAttr : public InheritableAttr {
unsigned unrollHint;

public:
  static OpenCLUnrollHintAttr *CreateImplicit(ASTContext &Ctx, unsigned UnrollHint, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLUnrollHintAttr(Ctx, CommonInfo, UnrollHint);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLUnrollHintAttr *Create(ASTContext &Ctx, unsigned UnrollHint, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OpenCLUnrollHintAttr(Ctx, CommonInfo, UnrollHint);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OpenCLUnrollHintAttr *CreateImplicit(ASTContext &Ctx, unsigned UnrollHint, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, UnrollHint, I);
  }
  static OpenCLUnrollHintAttr *Create(ASTContext &Ctx, unsigned UnrollHint, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, UnrollHint, I);
  }
  OpenCLUnrollHintAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned UnrollHint
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OpenCLUnrollHint, false, false)
              , unrollHint(UnrollHint)
  {
  }

  OpenCLUnrollHintAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getUnrollHint() const {
    return unrollHint;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::OpenCLUnrollHint; }
};

class OptimizeNoneAttr : public InheritableAttr {
public:
  static OptimizeNoneAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OptimizeNoneAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OptimizeNoneAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OptimizeNoneAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OptimizeNoneAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OptimizeNoneAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OptimizeNoneAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::OptimizeNone, false, false)
  {
  }

  OptimizeNoneAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::OptimizeNone; }
};

class OverloadableAttr : public Attr {
public:
  static OverloadableAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OverloadableAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OverloadableAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OverloadableAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OverloadableAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OverloadableAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OverloadableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::Overloadable, false)
  {
  }

  OverloadableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Overloadable; }
};

class OverrideAttr : public InheritableAttr {
public:
  static OverrideAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OverrideAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OverrideAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OverrideAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OverrideAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static OverrideAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  OverrideAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Override, false, false)
  {
  }

  OverrideAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Override; }
};

class OwnerAttr : public InheritableAttr {
TypeSourceInfo * derefType;

public:
  static OwnerAttr *CreateImplicit(ASTContext &Ctx, TypeSourceInfo * DerefType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OwnerAttr(Ctx, CommonInfo, DerefType);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OwnerAttr *Create(ASTContext &Ctx, TypeSourceInfo * DerefType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OwnerAttr(Ctx, CommonInfo, DerefType);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OwnerAttr *CreateImplicit(ASTContext &Ctx, TypeSourceInfo * DerefType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, DerefType, I);
  }
  static OwnerAttr *Create(ASTContext &Ctx, TypeSourceInfo * DerefType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, DerefType, I);
  }
  OwnerAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , TypeSourceInfo * DerefType
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Owner, false, false)
              , derefType(DerefType)
  {
  }

  OwnerAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Owner, false, false)
              , derefType()
  {
  }

  OwnerAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  QualType getDerefType() const {
    return derefType->getType();
  }  TypeSourceInfo * getDerefTypeLoc() const {
    return derefType;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Owner; }
};

class OwnershipAttr : public InheritableAttr {
IdentifierInfo * module;

  unsigned args_Size;
  ParamIdx *args_;

public:
  enum Spelling {
    GNU_ownership_holds = 0,
    CXX11_clang_ownership_holds = 1,
    C2x_clang_ownership_holds = 2,
    GNU_ownership_returns = 3,
    CXX11_clang_ownership_returns = 4,
    C2x_clang_ownership_returns = 5,
    GNU_ownership_takes = 6,
    CXX11_clang_ownership_takes = 7,
    C2x_clang_ownership_takes = 8,
  SpellingNotCalculated = 15

  };

  static OwnershipAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Module, ParamIdx *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OwnershipAttr(Ctx, CommonInfo, Module, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static OwnershipAttr *Create(ASTContext &Ctx, IdentifierInfo * Module, ParamIdx *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) OwnershipAttr(Ctx, CommonInfo, Module, Args, ArgsSize);
    return A;
  }

  static OwnershipAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * Module, ParamIdx *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OwnershipAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Module, Args, ArgsSize, I);
  }
  static OwnershipAttr *Create(ASTContext &Ctx, IdentifierInfo * Module, ParamIdx *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, OwnershipAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Module, Args, ArgsSize, I);
  }
  OwnershipAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * Module
              , ParamIdx *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Ownership, false, false)
              , module(Module)
              , args_Size(ArgsSize), args_(new (Ctx, 16) ParamIdx[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  OwnershipAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * Module
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Ownership, false, false)
              , module(Module)
              , args_Size(0), args_(nullptr)
  {
  }

  OwnershipAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_ownership_holds;
    case 1: return CXX11_clang_ownership_holds;
    case 2: return C2x_clang_ownership_holds;
    case 3: return GNU_ownership_returns;
    case 4: return CXX11_clang_ownership_returns;
    case 5: return C2x_clang_ownership_returns;
    case 6: return GNU_ownership_takes;
    case 7: return CXX11_clang_ownership_takes;
    case 8: return C2x_clang_ownership_takes;
  }
  }
  bool isHolds() const { return getAttributeSpellingListIndex() == 0 ||
    getAttributeSpellingListIndex() == 1 ||
    getAttributeSpellingListIndex() == 2; }
  bool isReturns() const { return getAttributeSpellingListIndex() == 3 ||
    getAttributeSpellingListIndex() == 4 ||
    getAttributeSpellingListIndex() == 5; }
  bool isTakes() const { return getAttributeSpellingListIndex() == 6 ||
    getAttributeSpellingListIndex() == 7 ||
    getAttributeSpellingListIndex() == 8; }
  IdentifierInfo * getModule() const {
    return module;
  }

  typedef ParamIdx* args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }



    enum OwnershipKind { Holds, Returns, Takes };
    OwnershipKind getOwnKind() const {
      return isHolds() ? Holds :
             isTakes() ? Takes :
             Returns;
    }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::Ownership; }
};

class PackedAttr : public InheritableAttr {
public:
  static PackedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PackedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PackedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PackedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PackedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static PackedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  PackedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Packed, false, false)
  {
  }

  PackedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Packed; }
};

class ParamTypestateAttr : public InheritableAttr {
public:
  enum ConsumedState {
    Unknown,
    Consumed,
    Unconsumed
  };
private:
  ConsumedState paramState;

public:
  static ParamTypestateAttr *CreateImplicit(ASTContext &Ctx, ConsumedState ParamState, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ParamTypestateAttr(Ctx, CommonInfo, ParamState);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ParamTypestateAttr *Create(ASTContext &Ctx, ConsumedState ParamState, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ParamTypestateAttr(Ctx, CommonInfo, ParamState);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ParamTypestateAttr *CreateImplicit(ASTContext &Ctx, ConsumedState ParamState, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, ParamState, I);
  }
  static ParamTypestateAttr *Create(ASTContext &Ctx, ConsumedState ParamState, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, ParamState, I);
  }
  ParamTypestateAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ConsumedState ParamState
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ParamTypestate, false, false)
              , paramState(ParamState)
  {
  }

  ParamTypestateAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  ConsumedState getParamState() const {
    return paramState;
  }

  static bool ConvertStrToConsumedState(StringRef Val, ConsumedState &Out) {
    Optional<ConsumedState> R = llvm::StringSwitch<Optional<ConsumedState>>(Val)
      .Case("unknown", ParamTypestateAttr::Unknown)
      .Case("consumed", ParamTypestateAttr::Consumed)
      .Case("unconsumed", ParamTypestateAttr::Unconsumed)
      .Default(Optional<ConsumedState>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertConsumedStateToStr(ConsumedState Val) {
    switch(Val) {
    case ParamTypestateAttr::Unknown: return "unknown";
    case ParamTypestateAttr::Consumed: return "consumed";
    case ParamTypestateAttr::Unconsumed: return "unconsumed";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::ParamTypestate; }
};

class PascalAttr : public InheritableAttr {
public:
  static PascalAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PascalAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PascalAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PascalAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PascalAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static PascalAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  PascalAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Pascal, false, false)
  {
  }

  PascalAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Pascal; }
};

class PassObjectSizeAttr : public InheritableParamAttr {
int type;

public:
  enum Spelling {
    GNU_pass_object_size = 0,
    CXX11_clang_pass_object_size = 1,
    C2x_clang_pass_object_size = 2,
    GNU_pass_dynamic_object_size = 3,
    CXX11_clang_pass_dynamic_object_size = 4,
    C2x_clang_pass_dynamic_object_size = 5,
  SpellingNotCalculated = 15

  };

  static PassObjectSizeAttr *CreateImplicit(ASTContext &Ctx, int Type, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PassObjectSizeAttr(Ctx, CommonInfo, Type);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PassObjectSizeAttr *Create(ASTContext &Ctx, int Type, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PassObjectSizeAttr(Ctx, CommonInfo, Type);
    return A;
  }

  static PassObjectSizeAttr *CreateImplicit(ASTContext &Ctx, int Type, SourceRange Range, AttributeCommonInfo::Syntax Syntax, PassObjectSizeAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Type, I);
  }
  static PassObjectSizeAttr *Create(ASTContext &Ctx, int Type, SourceRange Range, AttributeCommonInfo::Syntax Syntax, PassObjectSizeAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Type, I);
  }
  PassObjectSizeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , int Type
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::PassObjectSize, false, false)
              , type(Type)
  {
  }

  PassObjectSizeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_pass_object_size;
    case 1: return CXX11_clang_pass_object_size;
    case 2: return C2x_clang_pass_object_size;
    case 3: return GNU_pass_dynamic_object_size;
    case 4: return CXX11_clang_pass_dynamic_object_size;
    case 5: return C2x_clang_pass_dynamic_object_size;
  }
  }
  bool isDynamic() const { return getAttributeSpellingListIndex() == 3 ||
    getAttributeSpellingListIndex() == 4 ||
    getAttributeSpellingListIndex() == 5; }
  int getType() const {
    return type;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::PassObjectSize; }
};

class PatchableFunctionEntryAttr : public InheritableAttr {
unsigned count;

int offset;

public:
  static PatchableFunctionEntryAttr *CreateImplicit(ASTContext &Ctx, unsigned Count, int Offset, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PatchableFunctionEntryAttr(Ctx, CommonInfo, Count, Offset);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PatchableFunctionEntryAttr *Create(ASTContext &Ctx, unsigned Count, int Offset, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PatchableFunctionEntryAttr(Ctx, CommonInfo, Count, Offset);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PatchableFunctionEntryAttr *CreateImplicit(ASTContext &Ctx, unsigned Count, int Offset, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Count, Offset, I);
  }
  static PatchableFunctionEntryAttr *Create(ASTContext &Ctx, unsigned Count, int Offset, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Count, Offset, I);
  }
  PatchableFunctionEntryAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned Count
              , int Offset
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PatchableFunctionEntry, false, false)
              , count(Count)
              , offset(Offset)
  {
  }

  PatchableFunctionEntryAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned Count
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PatchableFunctionEntry, false, false)
              , count(Count)
              , offset()
  {
  }

  PatchableFunctionEntryAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getCount() const {
    return count;
  }

  int getOffset() const {
    return offset;
  }

  static const int DefaultOffset = 0;



  static bool classof(const Attr *A) { return A->getKind() == attr::PatchableFunctionEntry; }
};

class PcsAttr : public InheritableAttr {
public:
  enum PCSType {
    AAPCS,
    AAPCS_VFP
  };
private:
  PCSType pCS;

public:
  static PcsAttr *CreateImplicit(ASTContext &Ctx, PCSType PCS, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PcsAttr(Ctx, CommonInfo, PCS);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PcsAttr *Create(ASTContext &Ctx, PCSType PCS, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PcsAttr(Ctx, CommonInfo, PCS);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PcsAttr *CreateImplicit(ASTContext &Ctx, PCSType PCS, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, PCS, I);
  }
  static PcsAttr *Create(ASTContext &Ctx, PCSType PCS, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, PCS, I);
  }
  PcsAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , PCSType PCS
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Pcs, false, false)
              , pCS(PCS)
  {
  }

  PcsAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  PCSType getPCS() const {
    return pCS;
  }

  static bool ConvertStrToPCSType(StringRef Val, PCSType &Out) {
    Optional<PCSType> R = llvm::StringSwitch<Optional<PCSType>>(Val)
      .Case("aapcs", PcsAttr::AAPCS)
      .Case("aapcs-vfp", PcsAttr::AAPCS_VFP)
      .Default(Optional<PCSType>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertPCSTypeToStr(PCSType Val) {
    switch(Val) {
    case PcsAttr::AAPCS: return "aapcs";
    case PcsAttr::AAPCS_VFP: return "aapcs-vfp";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::Pcs; }
};

class PointerAttr : public InheritableAttr {
TypeSourceInfo * derefType;

public:
  static PointerAttr *CreateImplicit(ASTContext &Ctx, TypeSourceInfo * DerefType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PointerAttr(Ctx, CommonInfo, DerefType);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PointerAttr *Create(ASTContext &Ctx, TypeSourceInfo * DerefType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PointerAttr(Ctx, CommonInfo, DerefType);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PointerAttr *CreateImplicit(ASTContext &Ctx, TypeSourceInfo * DerefType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, DerefType, I);
  }
  static PointerAttr *Create(ASTContext &Ctx, TypeSourceInfo * DerefType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, DerefType, I);
  }
  PointerAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , TypeSourceInfo * DerefType
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Pointer, false, false)
              , derefType(DerefType)
  {
  }

  PointerAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Pointer, false, false)
              , derefType()
  {
  }

  PointerAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  QualType getDerefType() const {
    return derefType->getType();
  }  TypeSourceInfo * getDerefTypeLoc() const {
    return derefType;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Pointer; }
};

class PragmaClangBSSSectionAttr : public InheritableAttr {
unsigned nameLength;
char *name;

public:
  static PragmaClangBSSSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangBSSSectionAttr(Ctx, CommonInfo, Name);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangBSSSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangBSSSectionAttr(Ctx, CommonInfo, Name);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangBSSSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Name, I);
  }
  static PragmaClangBSSSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Name, I);
  }
  PragmaClangBSSSectionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Name
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PragmaClangBSSSection, false, false)
              , nameLength(Name.size()),name(new (Ctx, 1) char[nameLength])
  {
      if (!Name.empty())
        std::memcpy(name, Name.data(), nameLength);
  }

  PragmaClangBSSSectionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getName() const {
    return llvm::StringRef(name, nameLength);
  }
  unsigned getNameLength() const {
    return nameLength;
  }
  void setName(ASTContext &C, llvm::StringRef S) {
    nameLength = S.size();
    this->name = new (C, 1) char [nameLength];
    if (!S.empty())
      std::memcpy(this->name, S.data(), nameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::PragmaClangBSSSection; }
};

class PragmaClangDataSectionAttr : public InheritableAttr {
unsigned nameLength;
char *name;

public:
  static PragmaClangDataSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangDataSectionAttr(Ctx, CommonInfo, Name);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangDataSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangDataSectionAttr(Ctx, CommonInfo, Name);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangDataSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Name, I);
  }
  static PragmaClangDataSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Name, I);
  }
  PragmaClangDataSectionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Name
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PragmaClangDataSection, false, false)
              , nameLength(Name.size()),name(new (Ctx, 1) char[nameLength])
  {
      if (!Name.empty())
        std::memcpy(name, Name.data(), nameLength);
  }

  PragmaClangDataSectionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getName() const {
    return llvm::StringRef(name, nameLength);
  }
  unsigned getNameLength() const {
    return nameLength;
  }
  void setName(ASTContext &C, llvm::StringRef S) {
    nameLength = S.size();
    this->name = new (C, 1) char [nameLength];
    if (!S.empty())
      std::memcpy(this->name, S.data(), nameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::PragmaClangDataSection; }
};

class PragmaClangRelroSectionAttr : public InheritableAttr {
unsigned nameLength;
char *name;

public:
  static PragmaClangRelroSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangRelroSectionAttr(Ctx, CommonInfo, Name);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangRelroSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangRelroSectionAttr(Ctx, CommonInfo, Name);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangRelroSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Name, I);
  }
  static PragmaClangRelroSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Name, I);
  }
  PragmaClangRelroSectionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Name
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PragmaClangRelroSection, false, false)
              , nameLength(Name.size()),name(new (Ctx, 1) char[nameLength])
  {
      if (!Name.empty())
        std::memcpy(name, Name.data(), nameLength);
  }

  PragmaClangRelroSectionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getName() const {
    return llvm::StringRef(name, nameLength);
  }
  unsigned getNameLength() const {
    return nameLength;
  }
  void setName(ASTContext &C, llvm::StringRef S) {
    nameLength = S.size();
    this->name = new (C, 1) char [nameLength];
    if (!S.empty())
      std::memcpy(this->name, S.data(), nameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::PragmaClangRelroSection; }
};

class PragmaClangRodataSectionAttr : public InheritableAttr {
unsigned nameLength;
char *name;

public:
  static PragmaClangRodataSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangRodataSectionAttr(Ctx, CommonInfo, Name);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangRodataSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangRodataSectionAttr(Ctx, CommonInfo, Name);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangRodataSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Name, I);
  }
  static PragmaClangRodataSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Name, I);
  }
  PragmaClangRodataSectionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Name
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PragmaClangRodataSection, false, false)
              , nameLength(Name.size()),name(new (Ctx, 1) char[nameLength])
  {
      if (!Name.empty())
        std::memcpy(name, Name.data(), nameLength);
  }

  PragmaClangRodataSectionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getName() const {
    return llvm::StringRef(name, nameLength);
  }
  unsigned getNameLength() const {
    return nameLength;
  }
  void setName(ASTContext &C, llvm::StringRef S) {
    nameLength = S.size();
    this->name = new (C, 1) char [nameLength];
    if (!S.empty())
      std::memcpy(this->name, S.data(), nameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::PragmaClangRodataSection; }
};

class PragmaClangTextSectionAttr : public InheritableAttr {
unsigned nameLength;
char *name;

public:
  static PragmaClangTextSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangTextSectionAttr(Ctx, CommonInfo, Name);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangTextSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PragmaClangTextSectionAttr(Ctx, CommonInfo, Name);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PragmaClangTextSectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Name, I);
  }
  static PragmaClangTextSectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Name, I);
  }
  PragmaClangTextSectionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Name
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PragmaClangTextSection, false, false)
              , nameLength(Name.size()),name(new (Ctx, 1) char[nameLength])
  {
      if (!Name.empty())
        std::memcpy(name, Name.data(), nameLength);
  }

  PragmaClangTextSectionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getName() const {
    return llvm::StringRef(name, nameLength);
  }
  unsigned getNameLength() const {
    return nameLength;
  }
  void setName(ASTContext &C, llvm::StringRef S) {
    nameLength = S.size();
    this->name = new (C, 1) char [nameLength];
    if (!S.empty())
      std::memcpy(this->name, S.data(), nameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::PragmaClangTextSection; }
};

class PreserveAllAttr : public InheritableAttr {
public:
  static PreserveAllAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PreserveAllAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PreserveAllAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PreserveAllAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PreserveAllAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static PreserveAllAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  PreserveAllAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PreserveAll, false, false)
  {
  }

  PreserveAllAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::PreserveAll; }
};

class PreserveMostAttr : public InheritableAttr {
public:
  static PreserveMostAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PreserveMostAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PreserveMostAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PreserveMostAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PreserveMostAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static PreserveMostAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  PreserveMostAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PreserveMost, false, false)
  {
  }

  PreserveMostAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::PreserveMost; }
};

class PtGuardedByAttr : public InheritableAttr {
Expr * arg;

public:
  static PtGuardedByAttr *CreateImplicit(ASTContext &Ctx, Expr * Arg, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PtGuardedByAttr(Ctx, CommonInfo, Arg);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PtGuardedByAttr *Create(ASTContext &Ctx, Expr * Arg, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PtGuardedByAttr(Ctx, CommonInfo, Arg);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PtGuardedByAttr *CreateImplicit(ASTContext &Ctx, Expr * Arg, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Arg, I);
  }
  static PtGuardedByAttr *Create(ASTContext &Ctx, Expr * Arg, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Arg, I);
  }
  PtGuardedByAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * Arg
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PtGuardedBy, true, true)
              , arg(Arg)
  {
  }

  PtGuardedByAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getArg() const {
    return arg;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::PtGuardedBy; }
};

class PtGuardedVarAttr : public InheritableAttr {
public:
  static PtGuardedVarAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PtGuardedVarAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PtGuardedVarAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PtGuardedVarAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PtGuardedVarAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static PtGuardedVarAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  PtGuardedVarAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::PtGuardedVar, false, false)
  {
  }

  PtGuardedVarAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::PtGuardedVar; }
};

class Ptr32Attr : public TypeAttr {
public:
  static Ptr32Attr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) Ptr32Attr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static Ptr32Attr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) Ptr32Attr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static Ptr32Attr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static Ptr32Attr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  Ptr32Attr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::Ptr32, false)
  {
  }

  Ptr32Attr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Ptr32; }
};

class Ptr64Attr : public TypeAttr {
public:
  static Ptr64Attr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) Ptr64Attr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static Ptr64Attr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) Ptr64Attr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static Ptr64Attr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static Ptr64Attr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  Ptr64Attr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::Ptr64, false)
  {
  }

  Ptr64Attr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Ptr64; }
};

class PureAttr : public InheritableAttr {
public:
  static PureAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PureAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PureAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) PureAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static PureAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static PureAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  PureAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Pure, false, false)
  {
  }

  PureAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Pure; }
};

class RISCVInterruptAttr : public InheritableAttr {
public:
  enum InterruptType {
    user,
    supervisor,
    machine
  };
private:
  InterruptType interrupt;

public:
  static RISCVInterruptAttr *CreateImplicit(ASTContext &Ctx, InterruptType Interrupt, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RISCVInterruptAttr(Ctx, CommonInfo, Interrupt);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static RISCVInterruptAttr *Create(ASTContext &Ctx, InterruptType Interrupt, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RISCVInterruptAttr(Ctx, CommonInfo, Interrupt);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static RISCVInterruptAttr *CreateImplicit(ASTContext &Ctx, InterruptType Interrupt, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Interrupt, I);
  }
  static RISCVInterruptAttr *Create(ASTContext &Ctx, InterruptType Interrupt, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Interrupt, I);
  }
  RISCVInterruptAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , InterruptType Interrupt
             )
    : InheritableAttr(Ctx, CommonInfo, attr::RISCVInterrupt, false, false)
              , interrupt(Interrupt)
  {
  }

  RISCVInterruptAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::RISCVInterrupt, false, false)
              , interrupt(InterruptType(0))
  {
  }

  RISCVInterruptAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  InterruptType getInterrupt() const {
    return interrupt;
  }

  static bool ConvertStrToInterruptType(StringRef Val, InterruptType &Out) {
    Optional<InterruptType> R = llvm::StringSwitch<Optional<InterruptType>>(Val)
      .Case("user", RISCVInterruptAttr::user)
      .Case("supervisor", RISCVInterruptAttr::supervisor)
      .Case("machine", RISCVInterruptAttr::machine)
      .Default(Optional<InterruptType>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertInterruptTypeToStr(InterruptType Val) {
    switch(Val) {
    case RISCVInterruptAttr::user: return "user";
    case RISCVInterruptAttr::supervisor: return "supervisor";
    case RISCVInterruptAttr::machine: return "machine";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::RISCVInterrupt; }
};

class RegCallAttr : public InheritableAttr {
public:
  static RegCallAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RegCallAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static RegCallAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RegCallAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static RegCallAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static RegCallAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  RegCallAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::RegCall, false, false)
  {
  }

  RegCallAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::RegCall; }
};

class ReinitializesAttr : public InheritableAttr {
public:
  static ReinitializesAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReinitializesAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReinitializesAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReinitializesAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReinitializesAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ReinitializesAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ReinitializesAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Reinitializes, false, false)
  {
  }

  ReinitializesAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Reinitializes; }
};

class ReleaseCapabilityAttr : public InheritableAttr {
  unsigned args_Size;
  Expr * *args_;

public:
  enum Spelling {
    GNU_release_capability = 0,
    CXX11_clang_release_capability = 1,
    GNU_release_shared_capability = 2,
    CXX11_clang_release_shared_capability = 3,
    GNU_release_generic_capability = 4,
    CXX11_clang_release_generic_capability = 5,
    GNU_unlock_function = 6,
    CXX11_clang_unlock_function = 7,
  SpellingNotCalculated = 15

  };

  static ReleaseCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReleaseCapabilityAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReleaseCapabilityAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReleaseCapabilityAttr(Ctx, CommonInfo, Args, ArgsSize);
    return A;
  }

  static ReleaseCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, ReleaseCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static ReleaseCapabilityAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, ReleaseCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Args, ArgsSize, I);
  }
  ReleaseCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ReleaseCapability, true, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  ReleaseCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ReleaseCapability, true, true)
              , args_Size(0), args_(nullptr)
  {
  }

  ReleaseCapabilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_release_capability;
    case 1: return CXX11_clang_release_capability;
    case 2: return GNU_release_shared_capability;
    case 3: return CXX11_clang_release_shared_capability;
    case 4: return GNU_release_generic_capability;
    case 5: return CXX11_clang_release_generic_capability;
    case 6: return GNU_unlock_function;
    case 7: return CXX11_clang_unlock_function;
  }
  }
  bool isShared() const { return getAttributeSpellingListIndex() == 2 ||
    getAttributeSpellingListIndex() == 3; }
  bool isGeneric() const { return getAttributeSpellingListIndex() == 4 ||
    getAttributeSpellingListIndex() == 5 ||
    getAttributeSpellingListIndex() == 6 ||
    getAttributeSpellingListIndex() == 7; }
  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::ReleaseCapability; }
};

class ReleaseHandleAttr : public InheritableParamAttr {
unsigned handleTypeLength;
char *handleType;

public:
  static ReleaseHandleAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef HandleType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReleaseHandleAttr(Ctx, CommonInfo, HandleType);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReleaseHandleAttr *Create(ASTContext &Ctx, llvm::StringRef HandleType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReleaseHandleAttr(Ctx, CommonInfo, HandleType);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReleaseHandleAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef HandleType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, HandleType, I);
  }
  static ReleaseHandleAttr *Create(ASTContext &Ctx, llvm::StringRef HandleType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, HandleType, I);
  }
  ReleaseHandleAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef HandleType
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::ReleaseHandle, false, false)
              , handleTypeLength(HandleType.size()),handleType(new (Ctx, 1) char[handleTypeLength])
  {
      if (!HandleType.empty())
        std::memcpy(handleType, HandleType.data(), handleTypeLength);
  }

  ReleaseHandleAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getHandleType() const {
    return llvm::StringRef(handleType, handleTypeLength);
  }
  unsigned getHandleTypeLength() const {
    return handleTypeLength;
  }
  void setHandleType(ASTContext &C, llvm::StringRef S) {
    handleTypeLength = S.size();
    this->handleType = new (C, 1) char [handleTypeLength];
    if (!S.empty())
      std::memcpy(this->handleType, S.data(), handleTypeLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ReleaseHandle; }
};

class RenderScriptKernelAttr : public Attr {
public:
  static RenderScriptKernelAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RenderScriptKernelAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static RenderScriptKernelAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RenderScriptKernelAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static RenderScriptKernelAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static RenderScriptKernelAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  RenderScriptKernelAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::RenderScriptKernel, false)
  {
  }

  RenderScriptKernelAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::RenderScriptKernel; }
};

class ReqdWorkGroupSizeAttr : public InheritableAttr {
unsigned xDim;

unsigned yDim;

unsigned zDim;

public:
  static ReqdWorkGroupSizeAttr *CreateImplicit(ASTContext &Ctx, unsigned XDim, unsigned YDim, unsigned ZDim, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReqdWorkGroupSizeAttr(Ctx, CommonInfo, XDim, YDim, ZDim);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReqdWorkGroupSizeAttr *Create(ASTContext &Ctx, unsigned XDim, unsigned YDim, unsigned ZDim, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReqdWorkGroupSizeAttr(Ctx, CommonInfo, XDim, YDim, ZDim);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReqdWorkGroupSizeAttr *CreateImplicit(ASTContext &Ctx, unsigned XDim, unsigned YDim, unsigned ZDim, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, XDim, YDim, ZDim, I);
  }
  static ReqdWorkGroupSizeAttr *Create(ASTContext &Ctx, unsigned XDim, unsigned YDim, unsigned ZDim, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, XDim, YDim, ZDim, I);
  }
  ReqdWorkGroupSizeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned XDim
              , unsigned YDim
              , unsigned ZDim
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ReqdWorkGroupSize, false, false)
              , xDim(XDim)
              , yDim(YDim)
              , zDim(ZDim)
  {
  }

  ReqdWorkGroupSizeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getXDim() const {
    return xDim;
  }

  unsigned getYDim() const {
    return yDim;
  }

  unsigned getZDim() const {
    return zDim;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::ReqdWorkGroupSize; }
};

class RequiresCapabilityAttr : public InheritableAttr {
  unsigned args_Size;
  Expr * *args_;

public:
  enum Spelling {
    GNU_requires_capability = 0,
    CXX11_clang_requires_capability = 1,
    GNU_exclusive_locks_required = 2,
    CXX11_clang_exclusive_locks_required = 3,
    GNU_requires_shared_capability = 4,
    CXX11_clang_requires_shared_capability = 5,
    GNU_shared_locks_required = 6,
    CXX11_clang_shared_locks_required = 7,
  SpellingNotCalculated = 15

  };

  static RequiresCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RequiresCapabilityAttr(Ctx, CommonInfo, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static RequiresCapabilityAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RequiresCapabilityAttr(Ctx, CommonInfo, Args, ArgsSize);
    return A;
  }

  static RequiresCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, RequiresCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Args, ArgsSize, I);
  }
  static RequiresCapabilityAttr *Create(ASTContext &Ctx, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, RequiresCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Args, ArgsSize, I);
  }
  RequiresCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::RequiresCapability, true, true)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  RequiresCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::RequiresCapability, true, true)
              , args_Size(0), args_(nullptr)
  {
  }

  RequiresCapabilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_requires_capability;
    case 1: return CXX11_clang_requires_capability;
    case 2: return GNU_exclusive_locks_required;
    case 3: return CXX11_clang_exclusive_locks_required;
    case 4: return GNU_requires_shared_capability;
    case 5: return CXX11_clang_requires_shared_capability;
    case 6: return GNU_shared_locks_required;
    case 7: return CXX11_clang_shared_locks_required;
  }
  }
  bool isShared() const { return getAttributeSpellingListIndex() == 4 ||
    getAttributeSpellingListIndex() == 5 ||
    getAttributeSpellingListIndex() == 6 ||
    getAttributeSpellingListIndex() == 7; }
  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::RequiresCapability; }
};

class RestrictAttr : public InheritableAttr {
public:
  enum Spelling {
    Declspec_restrict = 0,
    GNU_malloc = 1,
    CXX11_gnu_malloc = 2,
  SpellingNotCalculated = 15

  };

  static RestrictAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RestrictAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static RestrictAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) RestrictAttr(Ctx, CommonInfo);
    return A;
  }

  static RestrictAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, RestrictAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static RestrictAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, RestrictAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  RestrictAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Restrict, false, false)
  {
  }

  RestrictAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return Declspec_restrict;
    case 1: return GNU_malloc;
    case 2: return CXX11_gnu_malloc;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::Restrict; }
};

class ReturnTypestateAttr : public InheritableAttr {
public:
  enum ConsumedState {
    Unknown,
    Consumed,
    Unconsumed
  };
private:
  ConsumedState state;

public:
  static ReturnTypestateAttr *CreateImplicit(ASTContext &Ctx, ConsumedState State, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReturnTypestateAttr(Ctx, CommonInfo, State);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReturnTypestateAttr *Create(ASTContext &Ctx, ConsumedState State, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReturnTypestateAttr(Ctx, CommonInfo, State);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReturnTypestateAttr *CreateImplicit(ASTContext &Ctx, ConsumedState State, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, State, I);
  }
  static ReturnTypestateAttr *Create(ASTContext &Ctx, ConsumedState State, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, State, I);
  }
  ReturnTypestateAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ConsumedState State
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ReturnTypestate, false, false)
              , state(State)
  {
  }

  ReturnTypestateAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  ConsumedState getState() const {
    return state;
  }

  static bool ConvertStrToConsumedState(StringRef Val, ConsumedState &Out) {
    Optional<ConsumedState> R = llvm::StringSwitch<Optional<ConsumedState>>(Val)
      .Case("unknown", ReturnTypestateAttr::Unknown)
      .Case("consumed", ReturnTypestateAttr::Consumed)
      .Case("unconsumed", ReturnTypestateAttr::Unconsumed)
      .Default(Optional<ConsumedState>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertConsumedStateToStr(ConsumedState Val) {
    switch(Val) {
    case ReturnTypestateAttr::Unknown: return "unknown";
    case ReturnTypestateAttr::Consumed: return "consumed";
    case ReturnTypestateAttr::Unconsumed: return "unconsumed";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::ReturnTypestate; }
};

class ReturnsNonNullAttr : public InheritableAttr {
public:
  static ReturnsNonNullAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReturnsNonNullAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReturnsNonNullAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReturnsNonNullAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReturnsNonNullAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ReturnsNonNullAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ReturnsNonNullAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ReturnsNonNull, false, false)
  {
  }

  ReturnsNonNullAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ReturnsNonNull; }
};

class ReturnsTwiceAttr : public InheritableAttr {
public:
  static ReturnsTwiceAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReturnsTwiceAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReturnsTwiceAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ReturnsTwiceAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ReturnsTwiceAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ReturnsTwiceAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ReturnsTwiceAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ReturnsTwice, false, false)
  {
  }

  ReturnsTwiceAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ReturnsTwice; }
};

class SPtrAttr : public TypeAttr {
public:
  static SPtrAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SPtrAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SPtrAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SPtrAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SPtrAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static SPtrAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  SPtrAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::SPtr, false)
  {
  }

  SPtrAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::SPtr; }
};

class SYCLKernelAttr : public InheritableAttr {
public:
  static SYCLKernelAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SYCLKernelAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SYCLKernelAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SYCLKernelAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SYCLKernelAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static SYCLKernelAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  SYCLKernelAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::SYCLKernel, false, false)
  {
  }

  SYCLKernelAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::SYCLKernel; }
};

class ScopedLockableAttr : public InheritableAttr {
public:
  static ScopedLockableAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ScopedLockableAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ScopedLockableAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ScopedLockableAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ScopedLockableAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ScopedLockableAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ScopedLockableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ScopedLockable, false, false)
  {
  }

  ScopedLockableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ScopedLockable; }
};

class SectionAttr : public InheritableAttr {
unsigned nameLength;
char *name;

public:
  enum Spelling {
    GNU_section = 0,
    CXX11_gnu_section = 1,
    Declspec_allocate = 2,
  SpellingNotCalculated = 15

  };

  static SectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SectionAttr(Ctx, CommonInfo, Name);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SectionAttr(Ctx, CommonInfo, Name);
    return A;
  }

  static SectionAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax, SectionAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Name, I);
  }
  static SectionAttr *Create(ASTContext &Ctx, llvm::StringRef Name, SourceRange Range, AttributeCommonInfo::Syntax Syntax, SectionAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Name, I);
  }
  SectionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Name
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Section, false, false)
              , nameLength(Name.size()),name(new (Ctx, 1) char[nameLength])
  {
      if (!Name.empty())
        std::memcpy(name, Name.data(), nameLength);
  }

  SectionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_section;
    case 1: return CXX11_gnu_section;
    case 2: return Declspec_allocate;
  }
  }
  llvm::StringRef getName() const {
    return llvm::StringRef(name, nameLength);
  }
  unsigned getNameLength() const {
    return nameLength;
  }
  void setName(ASTContext &C, llvm::StringRef S) {
    nameLength = S.size();
    this->name = new (C, 1) char [nameLength];
    if (!S.empty())
      std::memcpy(this->name, S.data(), nameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Section; }
};

class SelectAnyAttr : public InheritableAttr {
public:
  static SelectAnyAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SelectAnyAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SelectAnyAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SelectAnyAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SelectAnyAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static SelectAnyAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  SelectAnyAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::SelectAny, false, false)
  {
  }

  SelectAnyAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::SelectAny; }
};

class SentinelAttr : public InheritableAttr {
int sentinel;

int nullPos;

public:
  static SentinelAttr *CreateImplicit(ASTContext &Ctx, int Sentinel, int NullPos, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SentinelAttr(Ctx, CommonInfo, Sentinel, NullPos);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SentinelAttr *Create(ASTContext &Ctx, int Sentinel, int NullPos, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SentinelAttr(Ctx, CommonInfo, Sentinel, NullPos);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SentinelAttr *CreateImplicit(ASTContext &Ctx, int Sentinel, int NullPos, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Sentinel, NullPos, I);
  }
  static SentinelAttr *Create(ASTContext &Ctx, int Sentinel, int NullPos, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Sentinel, NullPos, I);
  }
  SentinelAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , int Sentinel
              , int NullPos
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Sentinel, false, false)
              , sentinel(Sentinel)
              , nullPos(NullPos)
  {
  }

  SentinelAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Sentinel, false, false)
              , sentinel()
              , nullPos()
  {
  }

  SentinelAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  int getSentinel() const {
    return sentinel;
  }

  static const int DefaultSentinel = 0;

  int getNullPos() const {
    return nullPos;
  }

  static const int DefaultNullPos = 0;



  static bool classof(const Attr *A) { return A->getKind() == attr::Sentinel; }
};

class SetTypestateAttr : public InheritableAttr {
public:
  enum ConsumedState {
    Unknown,
    Consumed,
    Unconsumed
  };
private:
  ConsumedState newState;

public:
  static SetTypestateAttr *CreateImplicit(ASTContext &Ctx, ConsumedState NewState, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SetTypestateAttr(Ctx, CommonInfo, NewState);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SetTypestateAttr *Create(ASTContext &Ctx, ConsumedState NewState, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SetTypestateAttr(Ctx, CommonInfo, NewState);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SetTypestateAttr *CreateImplicit(ASTContext &Ctx, ConsumedState NewState, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, NewState, I);
  }
  static SetTypestateAttr *Create(ASTContext &Ctx, ConsumedState NewState, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, NewState, I);
  }
  SetTypestateAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ConsumedState NewState
             )
    : InheritableAttr(Ctx, CommonInfo, attr::SetTypestate, false, false)
              , newState(NewState)
  {
  }

  SetTypestateAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  ConsumedState getNewState() const {
    return newState;
  }

  static bool ConvertStrToConsumedState(StringRef Val, ConsumedState &Out) {
    Optional<ConsumedState> R = llvm::StringSwitch<Optional<ConsumedState>>(Val)
      .Case("unknown", SetTypestateAttr::Unknown)
      .Case("consumed", SetTypestateAttr::Consumed)
      .Case("unconsumed", SetTypestateAttr::Unconsumed)
      .Default(Optional<ConsumedState>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertConsumedStateToStr(ConsumedState Val) {
    switch(Val) {
    case SetTypestateAttr::Unknown: return "unknown";
    case SetTypestateAttr::Consumed: return "consumed";
    case SetTypestateAttr::Unconsumed: return "unconsumed";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::SetTypestate; }
};

class SharedTrylockFunctionAttr : public InheritableAttr {
Expr * successValue;

  unsigned args_Size;
  Expr * *args_;

public:
  static SharedTrylockFunctionAttr *CreateImplicit(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SharedTrylockFunctionAttr(Ctx, CommonInfo, SuccessValue, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SharedTrylockFunctionAttr *Create(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SharedTrylockFunctionAttr(Ctx, CommonInfo, SuccessValue, Args, ArgsSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SharedTrylockFunctionAttr *CreateImplicit(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, SuccessValue, Args, ArgsSize, I);
  }
  static SharedTrylockFunctionAttr *Create(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, SuccessValue, Args, ArgsSize, I);
  }
  SharedTrylockFunctionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * SuccessValue
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::SharedTrylockFunction, true, true)
              , successValue(SuccessValue)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  SharedTrylockFunctionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * SuccessValue
             )
    : InheritableAttr(Ctx, CommonInfo, attr::SharedTrylockFunction, true, true)
              , successValue(SuccessValue)
              , args_Size(0), args_(nullptr)
  {
  }

  SharedTrylockFunctionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Expr * getSuccessValue() const {
    return successValue;
  }

  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::SharedTrylockFunction; }
};

class SpeculativeLoadHardeningAttr : public InheritableAttr {
public:
  static SpeculativeLoadHardeningAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SpeculativeLoadHardeningAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SpeculativeLoadHardeningAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SpeculativeLoadHardeningAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SpeculativeLoadHardeningAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static SpeculativeLoadHardeningAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  SpeculativeLoadHardeningAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::SpeculativeLoadHardening, false, false)
  {
  }

  SpeculativeLoadHardeningAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::SpeculativeLoadHardening; }
};

class StdCallAttr : public InheritableAttr {
public:
  static StdCallAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) StdCallAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static StdCallAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) StdCallAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static StdCallAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static StdCallAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  StdCallAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::StdCall, false, false)
  {
  }

  StdCallAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::StdCall; }
};

class SuppressAttr : public StmtAttr {
  unsigned diagnosticIdentifiers_Size;
  StringRef *diagnosticIdentifiers_;

public:
  static SuppressAttr *CreateImplicit(ASTContext &Ctx, StringRef *DiagnosticIdentifiers, unsigned DiagnosticIdentifiersSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SuppressAttr(Ctx, CommonInfo, DiagnosticIdentifiers, DiagnosticIdentifiersSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SuppressAttr *Create(ASTContext &Ctx, StringRef *DiagnosticIdentifiers, unsigned DiagnosticIdentifiersSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SuppressAttr(Ctx, CommonInfo, DiagnosticIdentifiers, DiagnosticIdentifiersSize);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SuppressAttr *CreateImplicit(ASTContext &Ctx, StringRef *DiagnosticIdentifiers, unsigned DiagnosticIdentifiersSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, DiagnosticIdentifiers, DiagnosticIdentifiersSize, I);
  }
  static SuppressAttr *Create(ASTContext &Ctx, StringRef *DiagnosticIdentifiers, unsigned DiagnosticIdentifiersSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, DiagnosticIdentifiers, DiagnosticIdentifiersSize, I);
  }
  SuppressAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , StringRef *DiagnosticIdentifiers, unsigned DiagnosticIdentifiersSize
             )
    : StmtAttr(Ctx, CommonInfo, attr::Suppress, false)
              , diagnosticIdentifiers_Size(DiagnosticIdentifiersSize), diagnosticIdentifiers_(new (Ctx, 16) StringRef[diagnosticIdentifiers_Size])
  {
    for (size_t I = 0, E = diagnosticIdentifiers_Size; I != E;
         ++I) {
      StringRef Ref = DiagnosticIdentifiers[I];
      if (!Ref.empty()) {
        char *Mem = new (Ctx, 1) char[Ref.size()];
        std::memcpy(Mem, Ref.data(), Ref.size());
        diagnosticIdentifiers_[I] = StringRef(Mem, Ref.size());
      }
    }
  }

  SuppressAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : StmtAttr(Ctx, CommonInfo, attr::Suppress, false)
              , diagnosticIdentifiers_Size(0), diagnosticIdentifiers_(nullptr)
  {
  }

  SuppressAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  typedef StringRef* diagnosticIdentifiers_iterator;
  diagnosticIdentifiers_iterator diagnosticIdentifiers_begin() const { return diagnosticIdentifiers_; }
  diagnosticIdentifiers_iterator diagnosticIdentifiers_end() const { return diagnosticIdentifiers_ + diagnosticIdentifiers_Size; }
  unsigned diagnosticIdentifiers_size() const { return diagnosticIdentifiers_Size; }
  llvm::iterator_range<diagnosticIdentifiers_iterator> diagnosticIdentifiers() const { return llvm::make_range(diagnosticIdentifiers_begin(), diagnosticIdentifiers_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::Suppress; }
};

class SwiftCallAttr : public InheritableAttr {
public:
  static SwiftCallAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SwiftCallAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SwiftCallAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SwiftCallAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SwiftCallAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static SwiftCallAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  SwiftCallAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::SwiftCall, false, false)
  {
  }

  SwiftCallAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::SwiftCall; }
};

class SwiftContextAttr : public ParameterABIAttr {
public:
  static SwiftContextAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SwiftContextAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SwiftContextAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SwiftContextAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SwiftContextAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static SwiftContextAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  SwiftContextAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : ParameterABIAttr(Ctx, CommonInfo, attr::SwiftContext, false, false)
  {
  }

  SwiftContextAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::SwiftContext; }
};

class SwiftErrorResultAttr : public ParameterABIAttr {
public:
  static SwiftErrorResultAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SwiftErrorResultAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SwiftErrorResultAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SwiftErrorResultAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SwiftErrorResultAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static SwiftErrorResultAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  SwiftErrorResultAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : ParameterABIAttr(Ctx, CommonInfo, attr::SwiftErrorResult, false, false)
  {
  }

  SwiftErrorResultAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::SwiftErrorResult; }
};

class SwiftIndirectResultAttr : public ParameterABIAttr {
public:
  static SwiftIndirectResultAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SwiftIndirectResultAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SwiftIndirectResultAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SwiftIndirectResultAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SwiftIndirectResultAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static SwiftIndirectResultAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  SwiftIndirectResultAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : ParameterABIAttr(Ctx, CommonInfo, attr::SwiftIndirectResult, false, false)
  {
  }

  SwiftIndirectResultAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::SwiftIndirectResult; }
};

class SysVABIAttr : public InheritableAttr {
public:
  static SysVABIAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SysVABIAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SysVABIAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) SysVABIAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static SysVABIAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static SysVABIAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  SysVABIAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::SysVABI, false, false)
  {
  }

  SysVABIAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::SysVABI; }
};

class TLSModelAttr : public InheritableAttr {
unsigned modelLength;
char *model;

public:
  static TLSModelAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Model, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TLSModelAttr(Ctx, CommonInfo, Model);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TLSModelAttr *Create(ASTContext &Ctx, llvm::StringRef Model, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TLSModelAttr(Ctx, CommonInfo, Model);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TLSModelAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Model, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Model, I);
  }
  static TLSModelAttr *Create(ASTContext &Ctx, llvm::StringRef Model, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Model, I);
  }
  TLSModelAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Model
             )
    : InheritableAttr(Ctx, CommonInfo, attr::TLSModel, false, false)
              , modelLength(Model.size()),model(new (Ctx, 1) char[modelLength])
  {
      if (!Model.empty())
        std::memcpy(model, Model.data(), modelLength);
  }

  TLSModelAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getModel() const {
    return llvm::StringRef(model, modelLength);
  }
  unsigned getModelLength() const {
    return modelLength;
  }
  void setModel(ASTContext &C, llvm::StringRef S) {
    modelLength = S.size();
    this->model = new (C, 1) char [modelLength];
    if (!S.empty())
      std::memcpy(this->model, S.data(), modelLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::TLSModel; }
};

class TargetAttr : public InheritableAttr {
unsigned featuresStrLength;
char *featuresStr;

public:
  static TargetAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef FeaturesStr, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TargetAttr(Ctx, CommonInfo, FeaturesStr);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TargetAttr *Create(ASTContext &Ctx, llvm::StringRef FeaturesStr, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TargetAttr(Ctx, CommonInfo, FeaturesStr);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TargetAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef FeaturesStr, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, FeaturesStr, I);
  }
  static TargetAttr *Create(ASTContext &Ctx, llvm::StringRef FeaturesStr, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, FeaturesStr, I);
  }
  TargetAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef FeaturesStr
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Target, false, false)
              , featuresStrLength(FeaturesStr.size()),featuresStr(new (Ctx, 1) char[featuresStrLength])
  {
      if (!FeaturesStr.empty())
        std::memcpy(featuresStr, FeaturesStr.data(), featuresStrLength);
  }

  TargetAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getFeaturesStr() const {
    return llvm::StringRef(featuresStr, featuresStrLength);
  }
  unsigned getFeaturesStrLength() const {
    return featuresStrLength;
  }
  void setFeaturesStr(ASTContext &C, llvm::StringRef S) {
    featuresStrLength = S.size();
    this->featuresStr = new (C, 1) char [featuresStrLength];
    if (!S.empty())
      std::memcpy(this->featuresStr, S.data(), featuresStrLength);
  }


    ParsedTargetAttr parse() const {
      return parse(getFeaturesStr());
    }

    StringRef getArchitecture() const {
      StringRef Features = getFeaturesStr();
      if (Features == "default") return {};

      SmallVector<StringRef, 1> AttrFeatures;
      Features.split(AttrFeatures, ",");

      for (auto &Feature : AttrFeatures) {
        Feature = Feature.trim();
        if (Feature.startswith("arch="))
          return Feature.drop_front(sizeof("arch=") - 1);
      }
      return "";
    }

    // Gets the list of features as simple string-refs with no +/- or 'no-'.
    // Only adds the items to 'Out' that are additions.
    void getAddedFeatures(llvm::SmallVectorImpl<StringRef> &Out) const {
      StringRef Features = getFeaturesStr();
      if (Features == "default") return;

      SmallVector<StringRef, 1> AttrFeatures;
      Features.split(AttrFeatures, ",");

      for (auto &Feature : AttrFeatures) {
        Feature = Feature.trim();

        if (!Feature.startswith("no-") && !Feature.startswith("arch=") &&
            !Feature.startswith("fpmath=") && !Feature.startswith("tune="))
          Out.push_back(Feature);
      }
    }

    template<class Compare>
    ParsedTargetAttr parse(Compare cmp) const {
      ParsedTargetAttr Attrs = parse();
      llvm::sort(std::begin(Attrs.Features), std::end(Attrs.Features), cmp);
      return Attrs;
    }

    bool isDefaultVersion() const { return getFeaturesStr() == "default"; }

    static ParsedTargetAttr parse(StringRef Features) {
      ParsedTargetAttr Ret;
      if (Features == "default") return Ret;
      SmallVector<StringRef, 1> AttrFeatures;
      Features.split(AttrFeatures, ",");

      // Grab the various features and prepend a "+" to turn on the feature to
      // the backend and add them to our existing set of features.
      for (auto &Feature : AttrFeatures) {
        // Go ahead and trim whitespace rather than either erroring or
        // accepting it weirdly.
        Feature = Feature.trim();

        // We don't support cpu tuning this way currently.
        // TODO: Support the fpmath option. It will require checking
        // overall feature validity for the function with the rest of the
        // attributes on the function.
        if (Feature.startswith("fpmath=") || Feature.startswith("tune="))
          continue;

        if (Feature.startswith("branch-protection=")) {
          Ret.BranchProtection = Feature.split('=').second.trim();
          continue;
        }

        // While we're here iterating check for a different target cpu.
        if (Feature.startswith("arch=")) {
          if (!Ret.Architecture.empty())
            Ret.DuplicateArchitecture = true;
          else
            Ret.Architecture = Feature.split("=").second.trim();
        } else if (Feature.startswith("no-"))
          Ret.Features.push_back("-" + Feature.split("-").second.str());
        else
          Ret.Features.push_back("+" + Feature.str());
      }
      return Ret;
    }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::Target; }
};

class TestTypestateAttr : public InheritableAttr {
public:
  enum ConsumedState {
    Consumed,
    Unconsumed
  };
private:
  ConsumedState testState;

public:
  static TestTypestateAttr *CreateImplicit(ASTContext &Ctx, ConsumedState TestState, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TestTypestateAttr(Ctx, CommonInfo, TestState);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TestTypestateAttr *Create(ASTContext &Ctx, ConsumedState TestState, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TestTypestateAttr(Ctx, CommonInfo, TestState);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TestTypestateAttr *CreateImplicit(ASTContext &Ctx, ConsumedState TestState, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, TestState, I);
  }
  static TestTypestateAttr *Create(ASTContext &Ctx, ConsumedState TestState, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, TestState, I);
  }
  TestTypestateAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , ConsumedState TestState
             )
    : InheritableAttr(Ctx, CommonInfo, attr::TestTypestate, false, false)
              , testState(TestState)
  {
  }

  TestTypestateAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  ConsumedState getTestState() const {
    return testState;
  }

  static bool ConvertStrToConsumedState(StringRef Val, ConsumedState &Out) {
    Optional<ConsumedState> R = llvm::StringSwitch<Optional<ConsumedState>>(Val)
      .Case("consumed", TestTypestateAttr::Consumed)
      .Case("unconsumed", TestTypestateAttr::Unconsumed)
      .Default(Optional<ConsumedState>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertConsumedStateToStr(ConsumedState Val) {
    switch(Val) {
    case TestTypestateAttr::Consumed: return "consumed";
    case TestTypestateAttr::Unconsumed: return "unconsumed";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::TestTypestate; }
};

class ThisCallAttr : public InheritableAttr {
public:
  static ThisCallAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ThisCallAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ThisCallAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ThisCallAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ThisCallAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ThisCallAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ThisCallAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::ThisCall, false, false)
  {
  }

  ThisCallAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::ThisCall; }
};

class ThreadAttr : public Attr {
public:
  static ThreadAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ThreadAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ThreadAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) ThreadAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static ThreadAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static ThreadAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  ThreadAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : Attr(Ctx, CommonInfo, attr::Thread, false)
  {
  }

  ThreadAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Thread; }
};

class TransparentUnionAttr : public InheritableAttr {
public:
  static TransparentUnionAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TransparentUnionAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TransparentUnionAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TransparentUnionAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TransparentUnionAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static TransparentUnionAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  TransparentUnionAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::TransparentUnion, false, false)
  {
  }

  TransparentUnionAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::TransparentUnion; }
};

class TrivialABIAttr : public InheritableAttr {
public:
  static TrivialABIAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TrivialABIAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TrivialABIAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TrivialABIAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TrivialABIAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static TrivialABIAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  TrivialABIAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::TrivialABI, false, false)
  {
  }

  TrivialABIAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::TrivialABI; }
};

class TryAcquireCapabilityAttr : public InheritableAttr {
Expr * successValue;

  unsigned args_Size;
  Expr * *args_;

public:
  enum Spelling {
    GNU_try_acquire_capability = 0,
    CXX11_clang_try_acquire_capability = 1,
    GNU_try_acquire_shared_capability = 2,
    CXX11_clang_try_acquire_shared_capability = 3,
  SpellingNotCalculated = 15

  };

  static TryAcquireCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TryAcquireCapabilityAttr(Ctx, CommonInfo, SuccessValue, Args, ArgsSize);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TryAcquireCapabilityAttr *Create(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TryAcquireCapabilityAttr(Ctx, CommonInfo, SuccessValue, Args, ArgsSize);
    return A;
  }

  static TryAcquireCapabilityAttr *CreateImplicit(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, TryAcquireCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, SuccessValue, Args, ArgsSize, I);
  }
  static TryAcquireCapabilityAttr *Create(ASTContext &Ctx, Expr * SuccessValue, Expr * *Args, unsigned ArgsSize, SourceRange Range, AttributeCommonInfo::Syntax Syntax, TryAcquireCapabilityAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, SuccessValue, Args, ArgsSize, I);
  }
  TryAcquireCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * SuccessValue
              , Expr * *Args, unsigned ArgsSize
             )
    : InheritableAttr(Ctx, CommonInfo, attr::TryAcquireCapability, true, true)
              , successValue(SuccessValue)
              , args_Size(ArgsSize), args_(new (Ctx, 16) Expr *[args_Size])
  {
    std::copy(Args, Args + args_Size, args_);
  }

  TryAcquireCapabilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , Expr * SuccessValue
             )
    : InheritableAttr(Ctx, CommonInfo, attr::TryAcquireCapability, true, true)
              , successValue(SuccessValue)
              , args_Size(0), args_(nullptr)
  {
  }

  TryAcquireCapabilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_try_acquire_capability;
    case 1: return CXX11_clang_try_acquire_capability;
    case 2: return GNU_try_acquire_shared_capability;
    case 3: return CXX11_clang_try_acquire_shared_capability;
  }
  }
  bool isShared() const { return getAttributeSpellingListIndex() == 2 ||
    getAttributeSpellingListIndex() == 3; }
  Expr * getSuccessValue() const {
    return successValue;
  }

  typedef Expr ** args_iterator;
  args_iterator args_begin() const { return args_; }
  args_iterator args_end() const { return args_ + args_Size; }
  unsigned args_size() const { return args_Size; }
  llvm::iterator_range<args_iterator> args() const { return llvm::make_range(args_begin(), args_end()); }




  static bool classof(const Attr *A) { return A->getKind() == attr::TryAcquireCapability; }
};

class TypeNonNullAttr : public TypeAttr {
public:
  static TypeNonNullAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeNonNullAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeNonNullAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeNonNullAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeNonNullAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static TypeNonNullAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  TypeNonNullAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::TypeNonNull, false)
  {
  }

  TypeNonNullAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::TypeNonNull; }
};

class TypeNullUnspecifiedAttr : public TypeAttr {
public:
  static TypeNullUnspecifiedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeNullUnspecifiedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeNullUnspecifiedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeNullUnspecifiedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeNullUnspecifiedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static TypeNullUnspecifiedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  TypeNullUnspecifiedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::TypeNullUnspecified, false)
  {
  }

  TypeNullUnspecifiedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::TypeNullUnspecified; }
};

class TypeNullableAttr : public TypeAttr {
public:
  static TypeNullableAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeNullableAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeNullableAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeNullableAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeNullableAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static TypeNullableAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  TypeNullableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::TypeNullable, false)
  {
  }

  TypeNullableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::TypeNullable; }
};

class TypeTagForDatatypeAttr : public InheritableAttr {
IdentifierInfo * argumentKind;

TypeSourceInfo * matchingCType;

bool layoutCompatible;

bool mustBeNull;

public:
  static TypeTagForDatatypeAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * ArgumentKind, TypeSourceInfo * MatchingCType, bool LayoutCompatible, bool MustBeNull, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeTagForDatatypeAttr(Ctx, CommonInfo, ArgumentKind, MatchingCType, LayoutCompatible, MustBeNull);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeTagForDatatypeAttr *Create(ASTContext &Ctx, IdentifierInfo * ArgumentKind, TypeSourceInfo * MatchingCType, bool LayoutCompatible, bool MustBeNull, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeTagForDatatypeAttr(Ctx, CommonInfo, ArgumentKind, MatchingCType, LayoutCompatible, MustBeNull);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeTagForDatatypeAttr *CreateImplicit(ASTContext &Ctx, IdentifierInfo * ArgumentKind, TypeSourceInfo * MatchingCType, bool LayoutCompatible, bool MustBeNull, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, ArgumentKind, MatchingCType, LayoutCompatible, MustBeNull, I);
  }
  static TypeTagForDatatypeAttr *Create(ASTContext &Ctx, IdentifierInfo * ArgumentKind, TypeSourceInfo * MatchingCType, bool LayoutCompatible, bool MustBeNull, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, ArgumentKind, MatchingCType, LayoutCompatible, MustBeNull, I);
  }
  TypeTagForDatatypeAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , IdentifierInfo * ArgumentKind
              , TypeSourceInfo * MatchingCType
              , bool LayoutCompatible
              , bool MustBeNull
             )
    : InheritableAttr(Ctx, CommonInfo, attr::TypeTagForDatatype, false, false)
              , argumentKind(ArgumentKind)
              , matchingCType(MatchingCType)
              , layoutCompatible(LayoutCompatible)
              , mustBeNull(MustBeNull)
  {
  }

  TypeTagForDatatypeAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  IdentifierInfo * getArgumentKind() const {
    return argumentKind;
  }

  QualType getMatchingCType() const {
    return matchingCType->getType();
  }  TypeSourceInfo * getMatchingCTypeLoc() const {
    return matchingCType;
  }

  bool getLayoutCompatible() const {
    return layoutCompatible;
  }

  bool getMustBeNull() const {
    return mustBeNull;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::TypeTagForDatatype; }
};

class TypeVisibilityAttr : public InheritableAttr {
public:
  enum VisibilityType {
    Default,
    Hidden,
    Protected
  };
private:
  VisibilityType visibility;

public:
  static TypeVisibilityAttr *CreateImplicit(ASTContext &Ctx, VisibilityType Visibility, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeVisibilityAttr(Ctx, CommonInfo, Visibility);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeVisibilityAttr *Create(ASTContext &Ctx, VisibilityType Visibility, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) TypeVisibilityAttr(Ctx, CommonInfo, Visibility);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static TypeVisibilityAttr *CreateImplicit(ASTContext &Ctx, VisibilityType Visibility, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Visibility, I);
  }
  static TypeVisibilityAttr *Create(ASTContext &Ctx, VisibilityType Visibility, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Visibility, I);
  }
  TypeVisibilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , VisibilityType Visibility
             )
    : InheritableAttr(Ctx, CommonInfo, attr::TypeVisibility, false, false)
              , visibility(Visibility)
  {
  }

  TypeVisibilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  VisibilityType getVisibility() const {
    return visibility;
  }

  static bool ConvertStrToVisibilityType(StringRef Val, VisibilityType &Out) {
    Optional<VisibilityType> R = llvm::StringSwitch<Optional<VisibilityType>>(Val)
      .Case("default", TypeVisibilityAttr::Default)
      .Case("hidden", TypeVisibilityAttr::Hidden)
      .Case("internal", TypeVisibilityAttr::Hidden)
      .Case("protected", TypeVisibilityAttr::Protected)
      .Default(Optional<VisibilityType>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertVisibilityTypeToStr(VisibilityType Val) {
    switch(Val) {
    case TypeVisibilityAttr::Default: return "default";
    case TypeVisibilityAttr::Hidden: return "hidden";
    case TypeVisibilityAttr::Protected: return "protected";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::TypeVisibility; }
};

class UPtrAttr : public TypeAttr {
public:
  static UPtrAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UPtrAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UPtrAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UPtrAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UPtrAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static UPtrAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  UPtrAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : TypeAttr(Ctx, CommonInfo, attr::UPtr, false)
  {
  }

  UPtrAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::UPtr; }
};

class UnavailableAttr : public InheritableAttr {
unsigned messageLength;
char *message;

public:
  enum ImplicitReason {
    IR_None,
    IR_ARCForbiddenType,
    IR_ForbiddenWeak,
    IR_ARCForbiddenConversion,
    IR_ARCInitReturnsUnrelated,
    IR_ARCFieldWithOwnership
  };
private:
  ImplicitReason implicitReason;

public:
  static UnavailableAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Message, ImplicitReason ImplicitReason, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UnavailableAttr(Ctx, CommonInfo, Message, ImplicitReason);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UnavailableAttr *Create(ASTContext &Ctx, llvm::StringRef Message, ImplicitReason ImplicitReason, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UnavailableAttr(Ctx, CommonInfo, Message, ImplicitReason);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UnavailableAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Message, ImplicitReason ImplicitReason, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Message, ImplicitReason, I);
  }
  static UnavailableAttr *Create(ASTContext &Ctx, llvm::StringRef Message, ImplicitReason ImplicitReason, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Message, ImplicitReason, I);
  }
  static UnavailableAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Message, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UnavailableAttr(Ctx, CommonInfo, Message);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UnavailableAttr *Create(ASTContext &Ctx, llvm::StringRef Message, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UnavailableAttr(Ctx, CommonInfo, Message);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UnavailableAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Message, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Message, I);
  }
  static UnavailableAttr *Create(ASTContext &Ctx, llvm::StringRef Message, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Message, I);
  }
  UnavailableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Message
              , ImplicitReason ImplicitReason
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Unavailable, false, false)
              , messageLength(Message.size()),message(new (Ctx, 1) char[messageLength])
              , implicitReason(ImplicitReason)
  {
      if (!Message.empty())
        std::memcpy(message, Message.data(), messageLength);
  }

  UnavailableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Message
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Unavailable, false, false)
              , messageLength(Message.size()),message(new (Ctx, 1) char[messageLength])
              , implicitReason(ImplicitReason(0))
  {
      if (!Message.empty())
        std::memcpy(message, Message.data(), messageLength);
  }

  UnavailableAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Unavailable, false, false)
              , messageLength(0),message(nullptr)
              , implicitReason(ImplicitReason(0))
  {
  }

  UnavailableAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getMessage() const {
    return llvm::StringRef(message, messageLength);
  }
  unsigned getMessageLength() const {
    return messageLength;
  }
  void setMessage(ASTContext &C, llvm::StringRef S) {
    messageLength = S.size();
    this->message = new (C, 1) char [messageLength];
    if (!S.empty())
      std::memcpy(this->message, S.data(), messageLength);
  }

  ImplicitReason getImplicitReason() const {
    return implicitReason;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Unavailable; }
};

class UninitializedAttr : public InheritableAttr {
public:
  static UninitializedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UninitializedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UninitializedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UninitializedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UninitializedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static UninitializedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  UninitializedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Uninitialized, false, false)
  {
  }

  UninitializedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Uninitialized; }
};

class UnusedAttr : public InheritableAttr {
public:
  enum Spelling {
    CXX11_maybe_unused = 0,
    GNU_unused = 1,
    CXX11_gnu_unused = 2,
    C2x_maybe_unused = 3,
  SpellingNotCalculated = 15

  };

  static UnusedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UnusedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UnusedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UnusedAttr(Ctx, CommonInfo);
    return A;
  }

  static UnusedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, UnusedAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static UnusedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, UnusedAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  UnusedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Unused, false, false)
  {
  }

  UnusedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return CXX11_maybe_unused;
    case 1: return GNU_unused;
    case 2: return CXX11_gnu_unused;
    case 3: return C2x_maybe_unused;
  }
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::Unused; }
};

class UseHandleAttr : public InheritableParamAttr {
unsigned handleTypeLength;
char *handleType;

public:
  static UseHandleAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef HandleType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UseHandleAttr(Ctx, CommonInfo, HandleType);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UseHandleAttr *Create(ASTContext &Ctx, llvm::StringRef HandleType, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UseHandleAttr(Ctx, CommonInfo, HandleType);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UseHandleAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef HandleType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, HandleType, I);
  }
  static UseHandleAttr *Create(ASTContext &Ctx, llvm::StringRef HandleType, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, HandleType, I);
  }
  UseHandleAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef HandleType
             )
    : InheritableParamAttr(Ctx, CommonInfo, attr::UseHandle, false, false)
              , handleTypeLength(HandleType.size()),handleType(new (Ctx, 1) char[handleTypeLength])
  {
      if (!HandleType.empty())
        std::memcpy(handleType, HandleType.data(), handleTypeLength);
  }

  UseHandleAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getHandleType() const {
    return llvm::StringRef(handleType, handleTypeLength);
  }
  unsigned getHandleTypeLength() const {
    return handleTypeLength;
  }
  void setHandleType(ASTContext &C, llvm::StringRef S) {
    handleTypeLength = S.size();
    this->handleType = new (C, 1) char [handleTypeLength];
    if (!S.empty())
      std::memcpy(this->handleType, S.data(), handleTypeLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::UseHandle; }
};

class UsedAttr : public InheritableAttr {
public:
  static UsedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UsedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UsedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UsedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UsedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static UsedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  UsedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Used, false, false)
  {
  }

  UsedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Used; }
};

class UuidAttr : public InheritableAttr {
unsigned guidLength;
char *guid;

public:
  static UuidAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Guid, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UuidAttr(Ctx, CommonInfo, Guid);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UuidAttr *Create(ASTContext &Ctx, llvm::StringRef Guid, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) UuidAttr(Ctx, CommonInfo, Guid);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static UuidAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Guid, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Guid, I);
  }
  static UuidAttr *Create(ASTContext &Ctx, llvm::StringRef Guid, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Guid, I);
  }
  UuidAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Guid
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Uuid, false, false)
              , guidLength(Guid.size()),guid(new (Ctx, 1) char[guidLength])
  {
      if (!Guid.empty())
        std::memcpy(guid, Guid.data(), guidLength);
  }

  UuidAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getGuid() const {
    return llvm::StringRef(guid, guidLength);
  }
  unsigned getGuidLength() const {
    return guidLength;
  }
  void setGuid(ASTContext &C, llvm::StringRef S) {
    guidLength = S.size();
    this->guid = new (C, 1) char [guidLength];
    if (!S.empty())
      std::memcpy(this->guid, S.data(), guidLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::Uuid; }
};

class VecReturnAttr : public InheritableAttr {
public:
  static VecReturnAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) VecReturnAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static VecReturnAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) VecReturnAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static VecReturnAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static VecReturnAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  VecReturnAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::VecReturn, false, false)
  {
  }

  VecReturnAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::VecReturn; }
};

class VecTypeHintAttr : public InheritableAttr {
TypeSourceInfo * typeHint;

public:
  static VecTypeHintAttr *CreateImplicit(ASTContext &Ctx, TypeSourceInfo * TypeHint, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) VecTypeHintAttr(Ctx, CommonInfo, TypeHint);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static VecTypeHintAttr *Create(ASTContext &Ctx, TypeSourceInfo * TypeHint, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) VecTypeHintAttr(Ctx, CommonInfo, TypeHint);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static VecTypeHintAttr *CreateImplicit(ASTContext &Ctx, TypeSourceInfo * TypeHint, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, TypeHint, I);
  }
  static VecTypeHintAttr *Create(ASTContext &Ctx, TypeSourceInfo * TypeHint, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, TypeHint, I);
  }
  VecTypeHintAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , TypeSourceInfo * TypeHint
             )
    : InheritableAttr(Ctx, CommonInfo, attr::VecTypeHint, false, false)
              , typeHint(TypeHint)
  {
  }

  VecTypeHintAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  QualType getTypeHint() const {
    return typeHint->getType();
  }  TypeSourceInfo * getTypeHintLoc() const {
    return typeHint;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::VecTypeHint; }
};

class VectorCallAttr : public InheritableAttr {
public:
  static VectorCallAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) VectorCallAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static VectorCallAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) VectorCallAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static VectorCallAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static VectorCallAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  VectorCallAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::VectorCall, false, false)
  {
  }

  VectorCallAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::VectorCall; }
};

class VisibilityAttr : public InheritableAttr {
public:
  enum VisibilityType {
    Default,
    Hidden,
    Protected
  };
private:
  VisibilityType visibility;

public:
  static VisibilityAttr *CreateImplicit(ASTContext &Ctx, VisibilityType Visibility, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) VisibilityAttr(Ctx, CommonInfo, Visibility);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static VisibilityAttr *Create(ASTContext &Ctx, VisibilityType Visibility, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) VisibilityAttr(Ctx, CommonInfo, Visibility);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static VisibilityAttr *CreateImplicit(ASTContext &Ctx, VisibilityType Visibility, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Visibility, I);
  }
  static VisibilityAttr *Create(ASTContext &Ctx, VisibilityType Visibility, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Visibility, I);
  }
  VisibilityAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , VisibilityType Visibility
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Visibility, false, false)
              , visibility(Visibility)
  {
  }

  VisibilityAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  VisibilityType getVisibility() const {
    return visibility;
  }

  static bool ConvertStrToVisibilityType(StringRef Val, VisibilityType &Out) {
    Optional<VisibilityType> R = llvm::StringSwitch<Optional<VisibilityType>>(Val)
      .Case("default", VisibilityAttr::Default)
      .Case("hidden", VisibilityAttr::Hidden)
      .Case("internal", VisibilityAttr::Hidden)
      .Case("protected", VisibilityAttr::Protected)
      .Default(Optional<VisibilityType>());
    if (R) {
      Out = *R;
      return true;
    }
    return false;
  }

  static const char *ConvertVisibilityTypeToStr(VisibilityType Val) {
    switch(Val) {
    case VisibilityAttr::Default: return "default";
    case VisibilityAttr::Hidden: return "hidden";
    case VisibilityAttr::Protected: return "protected";
    }
    llvm_unreachable("No enumerator with that value");
  }


  static bool classof(const Attr *A) { return A->getKind() == attr::Visibility; }
};

class WarnUnusedAttr : public InheritableAttr {
public:
  static WarnUnusedAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WarnUnusedAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WarnUnusedAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WarnUnusedAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WarnUnusedAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static WarnUnusedAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  WarnUnusedAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WarnUnused, false, false)
  {
  }

  WarnUnusedAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::WarnUnused; }
};

class WarnUnusedResultAttr : public InheritableAttr {
unsigned messageLength;
char *message;

public:
  enum Spelling {
    CXX11_nodiscard = 0,
    C2x_nodiscard = 1,
    CXX11_clang_warn_unused_result = 2,
    GNU_warn_unused_result = 3,
    CXX11_gnu_warn_unused_result = 4,
  SpellingNotCalculated = 15

  };

  static WarnUnusedResultAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Message, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WarnUnusedResultAttr(Ctx, CommonInfo, Message);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WarnUnusedResultAttr *Create(ASTContext &Ctx, llvm::StringRef Message, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WarnUnusedResultAttr(Ctx, CommonInfo, Message);
    return A;
  }

  static WarnUnusedResultAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Message, SourceRange Range, AttributeCommonInfo::Syntax Syntax, WarnUnusedResultAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, Message, I);
  }
  static WarnUnusedResultAttr *Create(ASTContext &Ctx, llvm::StringRef Message, SourceRange Range, AttributeCommonInfo::Syntax Syntax, WarnUnusedResultAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, Message, I);
  }
  WarnUnusedResultAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Message
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WarnUnusedResult, false, false)
              , messageLength(Message.size()),message(new (Ctx, 1) char[messageLength])
  {
      if (!Message.empty())
        std::memcpy(message, Message.data(), messageLength);
  }

  WarnUnusedResultAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WarnUnusedResult, false, false)
              , messageLength(0),message(nullptr)
  {
  }

  WarnUnusedResultAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return CXX11_nodiscard;
    case 1: return C2x_nodiscard;
    case 2: return CXX11_clang_warn_unused_result;
    case 3: return GNU_warn_unused_result;
    case 4: return CXX11_gnu_warn_unused_result;
  }
  }
  llvm::StringRef getMessage() const {
    return llvm::StringRef(message, messageLength);
  }
  unsigned getMessageLength() const {
    return messageLength;
  }
  void setMessage(ASTContext &C, llvm::StringRef S) {
    messageLength = S.size();
    this->message = new (C, 1) char [messageLength];
    if (!S.empty())
      std::memcpy(this->message, S.data(), messageLength);
  }


    // Check whether this the C++11 nodiscard version, even in non C++11
    // spellings.
    bool IsCXX11NoDiscard() const {
      return this->getSemanticSpelling() == CXX11_nodiscard;
    }
  

  static bool classof(const Attr *A) { return A->getKind() == attr::WarnUnusedResult; }
};

class WeakAttr : public InheritableAttr {
public:
  static WeakAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WeakAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WeakAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WeakAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WeakAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static WeakAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  WeakAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::Weak, false, false)
  {
  }

  WeakAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::Weak; }
};

class WeakImportAttr : public InheritableAttr {
public:
  static WeakImportAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WeakImportAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WeakImportAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WeakImportAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WeakImportAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static WeakImportAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  WeakImportAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WeakImport, false, false)
  {
  }

  WeakImportAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::WeakImport; }
};

class WeakRefAttr : public InheritableAttr {
unsigned aliaseeLength;
char *aliasee;

public:
  static WeakRefAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Aliasee, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WeakRefAttr(Ctx, CommonInfo, Aliasee);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WeakRefAttr *Create(ASTContext &Ctx, llvm::StringRef Aliasee, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WeakRefAttr(Ctx, CommonInfo, Aliasee);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WeakRefAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef Aliasee, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, Aliasee, I);
  }
  static WeakRefAttr *Create(ASTContext &Ctx, llvm::StringRef Aliasee, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, Aliasee, I);
  }
  WeakRefAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef Aliasee
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WeakRef, false, false)
              , aliaseeLength(Aliasee.size()),aliasee(new (Ctx, 1) char[aliaseeLength])
  {
      if (!Aliasee.empty())
        std::memcpy(aliasee, Aliasee.data(), aliaseeLength);
  }

  WeakRefAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WeakRef, false, false)
              , aliaseeLength(0),aliasee(nullptr)
  {
  }

  WeakRefAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getAliasee() const {
    return llvm::StringRef(aliasee, aliaseeLength);
  }
  unsigned getAliaseeLength() const {
    return aliaseeLength;
  }
  void setAliasee(ASTContext &C, llvm::StringRef S) {
    aliaseeLength = S.size();
    this->aliasee = new (C, 1) char [aliaseeLength];
    if (!S.empty())
      std::memcpy(this->aliasee, S.data(), aliaseeLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::WeakRef; }
};

class WebAssemblyExportNameAttr : public InheritableAttr {
unsigned exportNameLength;
char *exportName;

public:
  static WebAssemblyExportNameAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef ExportName, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WebAssemblyExportNameAttr(Ctx, CommonInfo, ExportName);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WebAssemblyExportNameAttr *Create(ASTContext &Ctx, llvm::StringRef ExportName, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WebAssemblyExportNameAttr(Ctx, CommonInfo, ExportName);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WebAssemblyExportNameAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef ExportName, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, ExportName, I);
  }
  static WebAssemblyExportNameAttr *Create(ASTContext &Ctx, llvm::StringRef ExportName, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, ExportName, I);
  }
  WebAssemblyExportNameAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef ExportName
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WebAssemblyExportName, false, false)
              , exportNameLength(ExportName.size()),exportName(new (Ctx, 1) char[exportNameLength])
  {
      if (!ExportName.empty())
        std::memcpy(exportName, ExportName.data(), exportNameLength);
  }

  WebAssemblyExportNameAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getExportName() const {
    return llvm::StringRef(exportName, exportNameLength);
  }
  unsigned getExportNameLength() const {
    return exportNameLength;
  }
  void setExportName(ASTContext &C, llvm::StringRef S) {
    exportNameLength = S.size();
    this->exportName = new (C, 1) char [exportNameLength];
    if (!S.empty())
      std::memcpy(this->exportName, S.data(), exportNameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::WebAssemblyExportName; }
};

class WebAssemblyImportModuleAttr : public InheritableAttr {
unsigned importModuleLength;
char *importModule;

public:
  static WebAssemblyImportModuleAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef ImportModule, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WebAssemblyImportModuleAttr(Ctx, CommonInfo, ImportModule);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WebAssemblyImportModuleAttr *Create(ASTContext &Ctx, llvm::StringRef ImportModule, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WebAssemblyImportModuleAttr(Ctx, CommonInfo, ImportModule);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WebAssemblyImportModuleAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef ImportModule, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, ImportModule, I);
  }
  static WebAssemblyImportModuleAttr *Create(ASTContext &Ctx, llvm::StringRef ImportModule, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, ImportModule, I);
  }
  WebAssemblyImportModuleAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef ImportModule
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WebAssemblyImportModule, false, false)
              , importModuleLength(ImportModule.size()),importModule(new (Ctx, 1) char[importModuleLength])
  {
      if (!ImportModule.empty())
        std::memcpy(importModule, ImportModule.data(), importModuleLength);
  }

  WebAssemblyImportModuleAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getImportModule() const {
    return llvm::StringRef(importModule, importModuleLength);
  }
  unsigned getImportModuleLength() const {
    return importModuleLength;
  }
  void setImportModule(ASTContext &C, llvm::StringRef S) {
    importModuleLength = S.size();
    this->importModule = new (C, 1) char [importModuleLength];
    if (!S.empty())
      std::memcpy(this->importModule, S.data(), importModuleLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::WebAssemblyImportModule; }
};

class WebAssemblyImportNameAttr : public InheritableAttr {
unsigned importNameLength;
char *importName;

public:
  static WebAssemblyImportNameAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef ImportName, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WebAssemblyImportNameAttr(Ctx, CommonInfo, ImportName);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WebAssemblyImportNameAttr *Create(ASTContext &Ctx, llvm::StringRef ImportName, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WebAssemblyImportNameAttr(Ctx, CommonInfo, ImportName);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WebAssemblyImportNameAttr *CreateImplicit(ASTContext &Ctx, llvm::StringRef ImportName, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, ImportName, I);
  }
  static WebAssemblyImportNameAttr *Create(ASTContext &Ctx, llvm::StringRef ImportName, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, ImportName, I);
  }
  WebAssemblyImportNameAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , llvm::StringRef ImportName
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WebAssemblyImportName, false, false)
              , importNameLength(ImportName.size()),importName(new (Ctx, 1) char[importNameLength])
  {
      if (!ImportName.empty())
        std::memcpy(importName, ImportName.data(), importNameLength);
  }

  WebAssemblyImportNameAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  llvm::StringRef getImportName() const {
    return llvm::StringRef(importName, importNameLength);
  }
  unsigned getImportNameLength() const {
    return importNameLength;
  }
  void setImportName(ASTContext &C, llvm::StringRef S) {
    importNameLength = S.size();
    this->importName = new (C, 1) char [importNameLength];
    if (!S.empty())
      std::memcpy(this->importName, S.data(), importNameLength);
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::WebAssemblyImportName; }
};

class WorkGroupSizeHintAttr : public InheritableAttr {
unsigned xDim;

unsigned yDim;

unsigned zDim;

public:
  static WorkGroupSizeHintAttr *CreateImplicit(ASTContext &Ctx, unsigned XDim, unsigned YDim, unsigned ZDim, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WorkGroupSizeHintAttr(Ctx, CommonInfo, XDim, YDim, ZDim);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WorkGroupSizeHintAttr *Create(ASTContext &Ctx, unsigned XDim, unsigned YDim, unsigned ZDim, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) WorkGroupSizeHintAttr(Ctx, CommonInfo, XDim, YDim, ZDim);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static WorkGroupSizeHintAttr *CreateImplicit(ASTContext &Ctx, unsigned XDim, unsigned YDim, unsigned ZDim, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, XDim, YDim, ZDim, I);
  }
  static WorkGroupSizeHintAttr *Create(ASTContext &Ctx, unsigned XDim, unsigned YDim, unsigned ZDim, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, XDim, YDim, ZDim, I);
  }
  WorkGroupSizeHintAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned XDim
              , unsigned YDim
              , unsigned ZDim
             )
    : InheritableAttr(Ctx, CommonInfo, attr::WorkGroupSizeHint, false, false)
              , xDim(XDim)
              , yDim(YDim)
              , zDim(ZDim)
  {
  }

  WorkGroupSizeHintAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getXDim() const {
    return xDim;
  }

  unsigned getYDim() const {
    return yDim;
  }

  unsigned getZDim() const {
    return zDim;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::WorkGroupSizeHint; }
};

class X86ForceAlignArgPointerAttr : public InheritableAttr {
public:
  static X86ForceAlignArgPointerAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) X86ForceAlignArgPointerAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static X86ForceAlignArgPointerAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) X86ForceAlignArgPointerAttr(Ctx, CommonInfo);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static X86ForceAlignArgPointerAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, I);
  }
  static X86ForceAlignArgPointerAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, I);
  }
  X86ForceAlignArgPointerAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::X86ForceAlignArgPointer, false, false)
  {
  }

  X86ForceAlignArgPointerAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;


  static bool classof(const Attr *A) { return A->getKind() == attr::X86ForceAlignArgPointer; }
};

class XRayInstrumentAttr : public InheritableAttr {
public:
  enum Spelling {
    GNU_xray_always_instrument = 0,
    CXX11_clang_xray_always_instrument = 1,
    C2x_clang_xray_always_instrument = 2,
    GNU_xray_never_instrument = 3,
    CXX11_clang_xray_never_instrument = 4,
    C2x_clang_xray_never_instrument = 5,
  SpellingNotCalculated = 15

  };

  static XRayInstrumentAttr *CreateImplicit(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) XRayInstrumentAttr(Ctx, CommonInfo);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static XRayInstrumentAttr *Create(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) XRayInstrumentAttr(Ctx, CommonInfo);
    return A;
  }

  static XRayInstrumentAttr *CreateImplicit(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, XRayInstrumentAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return CreateImplicit(Ctx, I);
  }
  static XRayInstrumentAttr *Create(ASTContext &Ctx, SourceRange Range, AttributeCommonInfo::Syntax Syntax, XRayInstrumentAttr::Spelling S = static_cast<Spelling>(SpellingNotCalculated)) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax, S);
    return Create(Ctx, I);
  }
  XRayInstrumentAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
             )
    : InheritableAttr(Ctx, CommonInfo, attr::XRayInstrument, false, false)
  {
  }

  XRayInstrumentAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  Spelling getSemanticSpelling() const {
  switch (getAttributeSpellingListIndex()) {
    default: llvm_unreachable("Unknown spelling list index");
    case 0: return GNU_xray_always_instrument;
    case 1: return CXX11_clang_xray_always_instrument;
    case 2: return C2x_clang_xray_always_instrument;
    case 3: return GNU_xray_never_instrument;
    case 4: return CXX11_clang_xray_never_instrument;
    case 5: return C2x_clang_xray_never_instrument;
  }
  }
  bool alwaysXRayInstrument() const { return getAttributeSpellingListIndex() == 0 ||
    getAttributeSpellingListIndex() == 1 ||
    getAttributeSpellingListIndex() == 2; }
  bool neverXRayInstrument() const { return getAttributeSpellingListIndex() == 3 ||
    getAttributeSpellingListIndex() == 4 ||
    getAttributeSpellingListIndex() == 5; }


  static bool classof(const Attr *A) { return A->getKind() == attr::XRayInstrument; }
};

class XRayLogArgsAttr : public InheritableAttr {
unsigned argumentCount;

public:
  static XRayLogArgsAttr *CreateImplicit(ASTContext &Ctx, unsigned ArgumentCount, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) XRayLogArgsAttr(Ctx, CommonInfo, ArgumentCount);
    A->setImplicit(true);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static XRayLogArgsAttr *Create(ASTContext &Ctx, unsigned ArgumentCount, const AttributeCommonInfo &CommonInfo = {SourceRange{}}) {
    auto *A = new (Ctx) XRayLogArgsAttr(Ctx, CommonInfo, ArgumentCount);
    if (!A->isAttributeSpellingListCalculated() && !A->getAttrName())
      A->setAttributeSpellingListIndex(0);
    return A;
  }

  static XRayLogArgsAttr *CreateImplicit(ASTContext &Ctx, unsigned ArgumentCount, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return CreateImplicit(Ctx, ArgumentCount, I);
  }
  static XRayLogArgsAttr *Create(ASTContext &Ctx, unsigned ArgumentCount, SourceRange Range, AttributeCommonInfo::Syntax Syntax) {
    AttributeCommonInfo I(Range, NoSemaHandlerAttribute, Syntax);
    return Create(Ctx, ArgumentCount, I);
  }
  XRayLogArgsAttr(ASTContext &Ctx, const AttributeCommonInfo &CommonInfo
              , unsigned ArgumentCount
             )
    : InheritableAttr(Ctx, CommonInfo, attr::XRayLogArgs, false, false)
              , argumentCount(ArgumentCount)
  {
  }

  XRayLogArgsAttr *clone(ASTContext &C) const;
  void printPretty(raw_ostream &OS,
                   const PrintingPolicy &Policy) const;
  const char *getSpelling() const;
  unsigned getArgumentCount() const {
    return argumentCount;
  }



  static bool classof(const Attr *A) { return A->getKind() == attr::XRayLogArgs; }
};

#endif // LLVM_CLANG_ATTR_CLASSES_INC
