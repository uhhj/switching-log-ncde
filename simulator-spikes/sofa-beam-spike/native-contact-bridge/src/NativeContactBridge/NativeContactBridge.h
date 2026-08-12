#pragma once

#include <NativeContactBridge/api.h>

#include <sofa/core/CollisionModel.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>
#include <sofa/core/objectmodel/BaseComponent.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/type/vector.h>

namespace nativecontactbridge
{
class NATIVE_CONTACT_BRIDGE_API NativeContactBridge : public sofa::core::objectmodel::BaseComponent
{
public:
    SOFA_CLASS(NativeContactBridge, sofa::core::objectmodel::BaseComponent);

    using ContactVector = sofa::type::vector<sofa::core::collision::DetectionOutput>;
    using ContactId = sofa::core::collision::DetectionOutput::ContactId;
    using CollisionModelLink = sofa::core::objectmodel::SingleLink<
        NativeContactBridge, sofa::core::CollisionModel,
        sofa::core::objectmodel::BaseLink::FLAG_STOREPATH | sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK>;
    using NarrowPhaseLink = sofa::core::objectmodel::SingleLink<
        NativeContactBridge, sofa::core::collision::NarrowPhaseDetection,
        sofa::core::objectmodel::BaseLink::FLAG_STOREPATH | sofa::core::objectmodel::BaseLink::FLAG_STRONGLINK>;

    void init() override;
    void handleEvent(sofa::core::objectmodel::Event* event) override;

    CollisionModelLink l_beamModel;
    CollisionModelLink l_fixtureModel;
    NarrowPhaseLink l_narrowPhase;
    sofa::core::objectmodel::Data<bool> d_ready;
    sofa::core::objectmodel::Data<unsigned int> d_frameSerial;
    sofa::core::objectmodel::Data<unsigned int> d_contactCount;
    sofa::core::objectmodel::Data<sofa::type::vector<ContactId>> d_contactIds;
    sofa::core::objectmodel::Data<sofa::type::vector<sofa::Index>> d_beamElementIds;
    sofa::core::objectmodel::Data<sofa::type::vector<sofa::Index>> d_fixtureElementIds;
    sofa::core::objectmodel::Data<sofa::type::vector<SReal>> d_beamContactPoints;
    sofa::core::objectmodel::Data<sofa::type::vector<SReal>> d_fixtureContactPoints;
    sofa::core::objectmodel::Data<sofa::type::vector<SReal>> d_beamOutwardNormals;
    sofa::core::objectmodel::Data<sofa::type::vector<SReal>> d_detectionValues;

protected:
    NativeContactBridge();
    ~NativeContactBridge() override = default;

private:
    void clearOutputs();
    void captureNativeContacts();
};

void registerNativeContactBridge(sofa::core::ObjectFactory* factory);
}
