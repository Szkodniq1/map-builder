#ifndef OCCMETHODTYPE_H
#define OCCMETHODTYPE_H

#include "Defs/defs.h"
#include "3rdParty/tinyXML/tinyxml2.h"

namespace mapping {

class OccMethodType {
private:
    std::string configPath = "../../resources/config.xml";
public:
    MethodType type;
    int pointTreshold;

    OccMethodType();

    void initMethodType();
    void assignType(int typeNum);
};
}

#endif // OCCMETHODTYPE_H
