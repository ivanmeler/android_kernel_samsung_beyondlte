/*===- TableGen'erated file -------------------------------------*- C++ -*-===*\
|*                                                                            *|
|* Helper classes for BasicWriters                                            *|
|*                                                                            *|
|* Automatically generated file, do not edit!                                 *|
|*                                                                            *|
\*===----------------------------------------------------------------------===*/

template <class ValueType>
struct WriteDispatcher;
template <>
struct WriteDispatcher<llvm::APInt> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeAPInt(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<llvm::APSInt> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeAPSInt(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<ArrayType::ArraySizeModifier> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeArraySizeModifier(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<attr::Kind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeAttrKind(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<AutoTypeKeyword> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeAutoTypeKeyword(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<bool> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeBool(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<BuiltinType::Kind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeBuiltinTypeKind(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<CXXRecordDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeCXXRecordDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const CXXRecordDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeCXXRecordDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<CallingConv> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeCallingConv(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<ConceptDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeConceptDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const ConceptDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeConceptDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<Decl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const Decl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<DeclarationName> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeDeclarationName(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<DeclarationName::NameKind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeDeclarationNameKind(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<ElaboratedTypeKeyword> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeElaboratedTypeKeyword(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<FunctionProtoType::ExceptionSpecInfo> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeExceptionSpecInfo(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<Expr*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeExprRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const Expr*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeExprRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<FunctionProtoType::ExtParameterInfo> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeExtParameterInfo(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<FunctionDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeFunctionDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const FunctionDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeFunctionDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<IdentifierInfo*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeIdentifier(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const IdentifierInfo*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeIdentifier(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<NamedDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeNamedDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const NamedDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeNamedDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<NamespaceAliasDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeNamespaceAliasDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const NamespaceAliasDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeNamespaceAliasDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<NamespaceDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeNamespaceDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const NamespaceDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeNamespaceDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<NestedNameSpecifier *> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeNestedNameSpecifier(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<NestedNameSpecifier::SpecifierKind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeNestedNameSpecifierKind(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<ObjCProtocolDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeObjCProtocolDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const ObjCProtocolDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeObjCProtocolDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<ObjCTypeParamDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeObjCTypeParamDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const ObjCTypeParamDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeObjCTypeParamDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<OverloadedOperatorKind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeOverloadedOperatorKind(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<QualType> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeQualType(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<Qualifiers> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeQualifiers(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<RefQualifierKind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeRefQualifierKind(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<Selector> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeSelector(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<SourceLocation> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeSourceLocation(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<Stmt*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeStmtRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const Stmt*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeStmtRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<TagDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTagDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const TagDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTagDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<TemplateArgument> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateArgument(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<TemplateArgument::ArgKind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateArgumentKind(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<TemplateDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const TemplateDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<TemplateName> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateName(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<TemplateName::NameKind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateNameKind(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<TemplateTemplateParmDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateTemplateParmDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const TemplateTemplateParmDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateTemplateParmDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<TemplateTypeParmDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateTypeParmDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const TemplateTypeParmDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeTemplateTypeParmDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<uint32_t> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeUInt32(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<uint64_t> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeUInt64(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<UnaryTransformType::UTTKind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeUnaryTypeTransformKind(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<ValueDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeValueDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<const ValueDecl*> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeValueDeclRef(std::forward<Args>(args)...);
  }
};
template <>
struct WriteDispatcher<VectorType::VectorKind> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeVectorKind(std::forward<Args>(args)...);
  }
};
template <class T>
struct WriteDispatcher<llvm::ArrayRef<T>> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeArray(std::forward<Args>(args)...);
  }
};
template <class T>
struct WriteDispatcher<llvm::Optional<T>> {
  template <class BasicWriter, class... Args>
  static void write(BasicWriter &W, Args &&... args) {
    return W.writeOptional(std::forward<Args>(args)...);
  }
};

template <class ValueType>
struct PackOptionalValue;
template <>
struct PackOptionalValue<CXXRecordDecl*> {
  static CXXRecordDecl* pack(Optional<CXXRecordDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const CXXRecordDecl*> {
  static const CXXRecordDecl* pack(Optional<const CXXRecordDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<ConceptDecl*> {
  static ConceptDecl* pack(Optional<ConceptDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const ConceptDecl*> {
  static const ConceptDecl* pack(Optional<const ConceptDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<Decl*> {
  static Decl* pack(Optional<Decl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const Decl*> {
  static const Decl* pack(Optional<const Decl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<Expr*> {
  static Expr* pack(Optional<Expr*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const Expr*> {
  static const Expr* pack(Optional<const Expr*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<FunctionDecl*> {
  static FunctionDecl* pack(Optional<FunctionDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const FunctionDecl*> {
  static const FunctionDecl* pack(Optional<const FunctionDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<IdentifierInfo*> {
  static IdentifierInfo* pack(Optional<IdentifierInfo*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const IdentifierInfo*> {
  static const IdentifierInfo* pack(Optional<const IdentifierInfo*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<NamedDecl*> {
  static NamedDecl* pack(Optional<NamedDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const NamedDecl*> {
  static const NamedDecl* pack(Optional<const NamedDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<NamespaceAliasDecl*> {
  static NamespaceAliasDecl* pack(Optional<NamespaceAliasDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const NamespaceAliasDecl*> {
  static const NamespaceAliasDecl* pack(Optional<const NamespaceAliasDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<NamespaceDecl*> {
  static NamespaceDecl* pack(Optional<NamespaceDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const NamespaceDecl*> {
  static const NamespaceDecl* pack(Optional<const NamespaceDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<ObjCProtocolDecl*> {
  static ObjCProtocolDecl* pack(Optional<ObjCProtocolDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const ObjCProtocolDecl*> {
  static const ObjCProtocolDecl* pack(Optional<const ObjCProtocolDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<ObjCTypeParamDecl*> {
  static ObjCTypeParamDecl* pack(Optional<ObjCTypeParamDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const ObjCTypeParamDecl*> {
  static const ObjCTypeParamDecl* pack(Optional<const ObjCTypeParamDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<QualType> {
  static QualType pack(Optional<QualType> value) {
    return value ? *value : QualType();
  }
};
template <>
struct PackOptionalValue<Stmt*> {
  static Stmt* pack(Optional<Stmt*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const Stmt*> {
  static const Stmt* pack(Optional<const Stmt*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<TagDecl*> {
  static TagDecl* pack(Optional<TagDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const TagDecl*> {
  static const TagDecl* pack(Optional<const TagDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<TemplateDecl*> {
  static TemplateDecl* pack(Optional<TemplateDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const TemplateDecl*> {
  static const TemplateDecl* pack(Optional<const TemplateDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<TemplateName> {
  static TemplateName pack(Optional<TemplateName> value) {
    return value ? *value : TemplateName();
  }
};
template <>
struct PackOptionalValue<TemplateTemplateParmDecl*> {
  static TemplateTemplateParmDecl* pack(Optional<TemplateTemplateParmDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const TemplateTemplateParmDecl*> {
  static const TemplateTemplateParmDecl* pack(Optional<const TemplateTemplateParmDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<TemplateTypeParmDecl*> {
  static TemplateTypeParmDecl* pack(Optional<TemplateTypeParmDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const TemplateTypeParmDecl*> {
  static const TemplateTypeParmDecl* pack(Optional<const TemplateTypeParmDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<uint32_t> {
  static uint32_t pack(Optional<uint32_t> value) {
    return value ? *value + 1 : 0;
  }
};
template <>
struct PackOptionalValue<uint64_t> {
  static uint64_t pack(Optional<uint64_t> value) {
    return value ? *value + 1 : 0;
  }
};
template <>
struct PackOptionalValue<ValueDecl*> {
  static ValueDecl* pack(Optional<ValueDecl*> value) {
    return value ? *value : nullptr;
  }
};
template <>
struct PackOptionalValue<const ValueDecl*> {
  static const ValueDecl* pack(Optional<const ValueDecl*> value) {
    return value ? *value : nullptr;
  }
};

template <class Impl>
class BasicWriterBase {
protected:
  BasicWriterBase() {}
public:
  Impl &asImpl() { return static_cast<Impl&>(*this); }
  void writeArraySizeModifier(ArrayType::ArraySizeModifier value) {
    asImpl().writeEnum(value);
  }
  void writeAttrKind(attr::Kind value) {
    asImpl().writeEnum(value);
  }
  void writeAutoTypeKeyword(AutoTypeKeyword value) {
    asImpl().writeEnum(value);
  }
  void writeBuiltinTypeKind(BuiltinType::Kind value) {
    asImpl().writeEnum(value);
  }
  void writeCXXRecordDeclRef(const CXXRecordDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeCallingConv(CallingConv value) {
    asImpl().writeEnum(value);
  }
  void writeConceptDeclRef(const ConceptDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeDeclarationName(DeclarationName node) {
    auto &&subW = asImpl().writeObject();
    DeclarationName::NameKind kind = (node.getNameKind());
    subW.find("kind").writeDeclarationNameKind(kind);
    switch (kind) {
    case DeclarationName::Identifier: {
    const IdentifierInfo* identifier = ( node.getAsIdentifierInfo() );
    subW.find("identifier").writeIdentifier(identifier);
    return;
    }

    case DeclarationName::ObjCMultiArgSelector: {
    Selector selector = ( node.getObjCSelector() );
    subW.find("selector").writeSelector(selector);
    return;
    }

    case DeclarationName::CXXConstructorName: {
    QualType type = ( node.getCXXNameType() );
    subW.find("type").writeQualType(type);
    return;
    }

    case DeclarationName::CXXDestructorName: {
    QualType type = ( node.getCXXNameType() );
    subW.find("type").writeQualType(type);
    return;
    }

    case DeclarationName::CXXConversionFunctionName: {
    QualType type = ( node.getCXXNameType() );
    subW.find("type").writeQualType(type);
    return;
    }

    case DeclarationName::CXXDeductionGuideName: {
    const TemplateDecl* declaration = ( node.getCXXDeductionGuideTemplate() );
    subW.find("declaration").writeTemplateDeclRef(declaration);
    return;
    }

    case DeclarationName::CXXOperatorName: {
    OverloadedOperatorKind operatorKind = ( node.getCXXOverloadedOperator() );
    subW.find("operatorKind").writeOverloadedOperatorKind(operatorKind);
    return;
    }

    case DeclarationName::CXXLiteralOperatorName: {
    const IdentifierInfo* identifier = ( node.getCXXLiteralIdentifier() );
    subW.find("identifier").writeIdentifier(identifier);
    return;
    }

    case DeclarationName::CXXUsingDirective: {
    return;
    }

    case DeclarationName::ObjCZeroArgSelector: {
    Selector selector = ( node.getObjCSelector() );
    subW.find("selector").writeSelector(selector);
    return;
    }

    case DeclarationName::ObjCOneArgSelector: {
    Selector selector = ( node.getObjCSelector() );
    subW.find("selector").writeSelector(selector);
    return;
    }

    }
    llvm_unreachable("bad DeclarationName::NameKind");
  }
  void writeDeclarationNameKind(DeclarationName::NameKind value) {
    asImpl().writeEnum(value);
  }
  void writeElaboratedTypeKeyword(ElaboratedTypeKeyword value) {
    asImpl().writeEnum(value);
  }
  void writeExprRef(const Expr* value) {
    asImpl().writeStmtRef(value);
  }
  void writeFunctionDeclRef(const FunctionDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeNamedDeclRef(const NamedDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeNamespaceAliasDeclRef(const NamespaceAliasDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeNamespaceDeclRef(const NamespaceDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeNestedNameSpecifierKind(NestedNameSpecifier::SpecifierKind value) {
    asImpl().writeEnum(value);
  }
  void writeObjCProtocolDeclRef(const ObjCProtocolDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeObjCTypeParamDeclRef(const ObjCTypeParamDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeOverloadedOperatorKind(OverloadedOperatorKind value) {
    asImpl().writeEnum(value);
  }
  void writeRefQualifierKind(RefQualifierKind value) {
    asImpl().writeEnum(value);
  }
  void writeTagDeclRef(const TagDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeTemplateArgument(TemplateArgument node) {
    auto &&subW = asImpl().writeObject();
    TemplateArgument::ArgKind kind = (node.getKind());
    subW.find("kind").writeTemplateArgumentKind(kind);
    switch (kind) {
    case TemplateArgument::Null: {
    return;
    }

    case TemplateArgument::Type: {
    QualType type = ( node.getAsType() );
    subW.find("type").writeQualType(type);
    return;
    }

    case TemplateArgument::Declaration: {
    const ValueDecl* declaration = ( node.getAsDecl() );
    subW.find("declaration").writeValueDeclRef(declaration);
    QualType parameterType = ( node.getParamTypeForDecl() );
    subW.find("parameterType").writeQualType(parameterType);
    return;
    }

    case TemplateArgument::NullPtr: {
    QualType type = ( node.getNullPtrType() );
    subW.find("type").writeQualType(type);
    return;
    }

    case TemplateArgument::Integral: {
    llvm::APSInt value = ( node.getAsIntegral() );
    subW.find("value").writeAPSInt(value);
    QualType type = ( node.getIntegralType() );
    subW.find("type").writeQualType(type);
    return;
    }

    case TemplateArgument::Template: {
    TemplateName name = ( node.getAsTemplateOrTemplatePattern() );
    subW.find("name").writeTemplateName(name);
    return;
    }

    case TemplateArgument::TemplateExpansion: {
    TemplateName name = ( node.getAsTemplateOrTemplatePattern() );
    subW.find("name").writeTemplateName(name);
    llvm::Optional<uint32_t> numExpansions = (
      // Translate unsigned -> uint32_t just in case.
      node.getNumTemplateExpansions().map(
        [](unsigned i) { return uint32_t(i); })
    );
    subW.find("numExpansions").writeOptional(numExpansions);
    return;
    }

    case TemplateArgument::Expression: {
    const Expr* expression = ( node.getAsExpr() );
    subW.find("expression").writeExprRef(expression);
    return;
    }

    case TemplateArgument::Pack: {
    llvm::ArrayRef<TemplateArgument> elements = ( node.pack_elements() );
    subW.find("elements").writeArray(elements);
    return;
    }

    }
    llvm_unreachable("bad TemplateArgument::ArgKind");
  }
  void writeTemplateArgumentKind(TemplateArgument::ArgKind value) {
    asImpl().writeEnum(value);
  }
  void writeTemplateDeclRef(const TemplateDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeTemplateName(TemplateName node) {
    auto &&subW = asImpl().writeObject();
    TemplateName::NameKind kind = (node.getKind());
    subW.find("kind").writeTemplateNameKind(kind);
    switch (kind) {
    case TemplateName::Template: {
    const TemplateDecl* declaration = ( node.getAsTemplateDecl() );
    subW.find("declaration").writeTemplateDeclRef(declaration);
    return;
    }

    case TemplateName::OverloadedTemplate: {
    llvm::ArrayRef<const NamedDecl*> overloads = ( node.getAsOverloadedTemplate()->decls() );
    subW.find("overloads").writeArray(overloads);
    return;
    }

    case TemplateName::AssumedTemplate: {
    DeclarationName name = ( node.getAsAssumedTemplateName()->getDeclName() );
    subW.find("name").writeDeclarationName(name);
    return;
    }

    case TemplateName::QualifiedTemplate: {
    
    auto qtn = node.getAsQualifiedTemplateName();
  
    NestedNameSpecifier * qualifier = ( qtn->getQualifier() );
    subW.find("qualifier").writeNestedNameSpecifier(qualifier);
    bool hasTemplateKeyword = ( qtn->hasTemplateKeyword() );
    subW.find("hasTemplateKeyword").writeBool(hasTemplateKeyword);
    const TemplateDecl* declaration = ( qtn->getTemplateDecl() );
    subW.find("declaration").writeTemplateDeclRef(declaration);
    return;
    }

    case TemplateName::DependentTemplate: {
    
    auto dtn = node.getAsDependentTemplateName();
  
    NestedNameSpecifier * qualifier = ( dtn->getQualifier() );
    subW.find("qualifier").writeNestedNameSpecifier(qualifier);
    llvm::Optional<const IdentifierInfo*> identifier = ( makeOptionalFromPointer(
                    dtn->isIdentifier()
                      ? dtn->getIdentifier()
                      : nullptr) );
    subW.find("identifier").writeOptional(identifier);
    if ( !identifier ) {
    OverloadedOperatorKind operatorKind = ( dtn->getOperator() );
    subW.find("operatorKind").writeOverloadedOperatorKind(operatorKind);
    }
    return;
    }

    case TemplateName::SubstTemplateTemplateParm: {
    
    auto parm = node.getAsSubstTemplateTemplateParm();
  
    const TemplateTemplateParmDecl* parameter = ( parm->getParameter() );
    subW.find("parameter").writeTemplateTemplateParmDeclRef(parameter);
    TemplateName replacement = ( parm->getReplacement() );
    subW.find("replacement").writeTemplateName(replacement);
    return;
    }

    case TemplateName::SubstTemplateTemplateParmPack: {
    
    auto parm = node.getAsSubstTemplateTemplateParmPack();
  
    const TemplateTemplateParmDecl* parameterPack = ( parm->getParameterPack() );
    subW.find("parameterPack").writeTemplateTemplateParmDeclRef(parameterPack);
    TemplateArgument argumentPack = ( parm->getArgumentPack() );
    subW.find("argumentPack").writeTemplateArgument(argumentPack);
    return;
    }

    }
    llvm_unreachable("bad TemplateName::NameKind");
  }
  void writeTemplateNameKind(TemplateName::NameKind value) {
    asImpl().writeEnum(value);
  }
  void writeTemplateTemplateParmDeclRef(const TemplateTemplateParmDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeTemplateTypeParmDeclRef(const TemplateTypeParmDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeUnaryTypeTransformKind(UnaryTransformType::UTTKind value) {
    asImpl().writeEnum(value);
  }
  void writeValueDeclRef(const ValueDecl* value) {
    asImpl().writeDeclRef(value);
  }
  void writeVectorKind(VectorType::VectorKind value) {
    asImpl().writeEnum(value);
  }
};

