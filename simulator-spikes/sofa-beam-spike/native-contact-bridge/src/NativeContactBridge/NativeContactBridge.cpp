#include <NativeContactBridge/NativeContactBridge.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/simulation/CollisionEndEvent.h>

namespace nativecontactbridge
{
void registerNativeContactBridge(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData(
        "Expose native narrow-phase contact outputs through ordinary SOFA Data.")
        .add<NativeContactBridge>());
}

NativeContactBridge::NativeContactBridge()
    : l_beamModel(initLink("beamModel", "Beam collision model whose native contacts will be exported."))
    , l_fixtureModel(initLink("fixtureModel", "Fixture collision model whose native contacts will be exported."))
    , l_narrowPhase(initLink("narrowPhase", "NarrowPhaseDetection component."))
    , d_ready(initData(&d_ready, false, "ready", "Links resolved."))
    , d_frameSerial(initData(&d_frameSerial, 0u, "frameSerial", "Captured CollisionEndEvent frames."))
    , d_contactCount(initData(&d_contactCount, 0u, "contactCount", "Native contacts for the requested pair."))
    , d_contactIds(initData(&d_contactIds, "contactIds", "Unmodified DetectionOutput::ContactId values."))
    , d_beamElementIds(initData(&d_beamElementIds, "beamElementIds", "Native beam primitive IDs."))
    , d_fixtureElementIds(initData(&d_fixtureElementIds, "fixtureElementIds", "Native fixture primitive IDs."))
    , d_beamContactPoints(initData(&d_beamContactPoints, "beamContactPoints", "Flattened native beam-side points."))
    , d_fixtureContactPoints(initData(&d_fixtureContactPoints, "fixtureContactPoints", "Flattened native fixture-side points."))
    , d_beamOutwardNormals(initData(&d_beamOutwardNormals, "beamOutwardNormals", "Flattened normals outward from beam."))
    , d_detectionValues(initData(&d_detectionValues, "detectionValues", "Raw DetectionOutput values."))
{
    f_listening.setValue(true);
}

void NativeContactBridge::init()
{
    if (l_narrowPhase.get() == nullptr)
        l_narrowPhase.set(getContext()->get<sofa::core::collision::NarrowPhaseDetection>());

    const bool ready = l_beamModel.get() != nullptr && l_fixtureModel.get() != nullptr && l_narrowPhase.get() != nullptr;
    d_ready.setValue(ready);
    if (!ready)
        msg_error() << "NativeContactBridge requires beamModel, fixtureModel, and NarrowPhaseDetection.";
    clearOutputs();
}

void NativeContactBridge::clearOutputs()
{
    d_contactCount.setValue(0);
    d_contactIds.setValue({});
    d_beamElementIds.setValue({});
    d_fixtureElementIds.setValue({});
    d_beamContactPoints.setValue({});
    d_fixtureContactPoints.setValue({});
    d_beamOutwardNormals.setValue({});
    d_detectionValues.setValue({});
}

void NativeContactBridge::handleEvent(sofa::core::objectmodel::Event* event)
{
    if (sofa::simulation::CollisionEndEvent::checkEventType(event))
        captureNativeContacts();
}

void NativeContactBridge::captureNativeContacts()
{
    clearOutputs();
    const auto* beamModel = l_beamModel.get();
    const auto* fixtureModel = l_fixtureModel.get();
    const auto* narrowPhase = l_narrowPhase.get();
    if (beamModel == nullptr || fixtureModel == nullptr || narrowPhase == nullptr)
    {
        d_ready.setValue(false);
        return;
    }

    sofa::type::vector<ContactId> contactIds;
    sofa::type::vector<sofa::Index> beamIds, fixtureIds;
    sofa::type::vector<SReal> beamPoints, fixturePoints, beamNormals, detectionValues;
    for (const auto& entry : narrowPhase->getDetectionOutputs())
    {
        const auto* firstModel = entry.first.first;
        const auto* secondModel = entry.first.second;
        if (!((firstModel == beamModel && secondModel == fixtureModel)
              || (firstModel == fixtureModel && secondModel == beamModel)))
            continue;
        const auto* contacts = dynamic_cast<const ContactVector*>(entry.second);
        if (contacts == nullptr)
            continue;
        for (const auto& contact : *contacts)
        {
            const bool beamIsFirst = contact.elem.first.getCollisionModel() == beamModel;
            const auto beamElement = beamIsFirst ? contact.elem.first.getIndex() : contact.elem.second.getIndex();
            const auto fixtureElement = beamIsFirst ? contact.elem.second.getIndex() : contact.elem.first.getIndex();
            const auto& beamPoint = beamIsFirst ? contact.point[0] : contact.point[1];
            const auto& fixturePoint = beamIsFirst ? contact.point[1] : contact.point[0];
            auto beamNormal = contact.normal;
            if (!beamIsFirst)
                for (unsigned int axis = 0; axis < 3; ++axis) beamNormal[axis] = -beamNormal[axis];

            contactIds.push_back(contact.id);
            beamIds.push_back(beamElement);
            fixtureIds.push_back(fixtureElement);
            for (unsigned int axis = 0; axis < 3; ++axis)
            {
                beamPoints.push_back(beamPoint[axis]);
                fixturePoints.push_back(fixturePoint[axis]);
                beamNormals.push_back(beamNormal[axis]);
            }
            detectionValues.push_back(static_cast<SReal>(contact.value));
        }
    }
    d_contactIds.setValue(contactIds);
    d_beamElementIds.setValue(beamIds);
    d_fixtureElementIds.setValue(fixtureIds);
    d_beamContactPoints.setValue(beamPoints);
    d_fixtureContactPoints.setValue(fixturePoints);
    d_beamOutwardNormals.setValue(beamNormals);
    d_detectionValues.setValue(detectionValues);
    d_contactCount.setValue(static_cast<unsigned int>(beamIds.size()));
    d_frameSerial.setValue(d_frameSerial.getValue() + 1);
}
}
