/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "testModels.h"

#include <iDynTree/Core/TestUtils.h>


#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/ModelIO/ModelExporter.h>


#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <fstream>

using namespace iDynTree;

unsigned int getNrOfVisuals(const iDynTree::Model& model)
{
    unsigned int nrOfVisuals = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index) {
        nrOfVisuals += model.visualSolidShapes().linkSolidShapes[index].size();
    }
    return nrOfVisuals;
}

unsigned int getNrOfCollisions(const iDynTree::Model& model)
{
    unsigned int nrOfCollisions = 0;
    for (LinkIndex index = 0; index < model.getNrOfLinks(); ++index) {
        nrOfCollisions += model.collisionSolidShapes().linkSolidShapes[index].size();
    }
    return nrOfCollisions;
}

void checkSolidAreEqual(SolidShape* solid, SolidShape* solidCheck)
{
    ASSERT_IS_TRUE(solid->isBox() == solidCheck->isBox());
    ASSERT_IS_TRUE(solid->isCylinder() == solidCheck->isCylinder());
    ASSERT_IS_TRUE(solid->isSphere() == solidCheck->isSphere());
    ASSERT_IS_TRUE(solid->isExternalMesh() == solidCheck->isExternalMesh());

    ASSERT_IS_TRUE(solid->name == solidCheck->name);
    ASSERT_IS_TRUE(solid->nameIsValid == solidCheck->nameIsValid);

    ASSERT_EQUAL_TRANSFORM(solid->link_H_geometry, solidCheck->link_H_geometry);

    if (solid->isBox()) {
        const Box* box = solid->asBox();
        const Box* boxCheck = solidCheck->asBox();
        ASSERT_EQUAL_DOUBLE(box->x, boxCheck->x);
        ASSERT_EQUAL_DOUBLE(box->y, boxCheck->y);
        ASSERT_EQUAL_DOUBLE(box->z, boxCheck->z);

    } else if (solid->isCylinder()) {
        const Cylinder* cylinder = solid->asCylinder();
        const Cylinder* cylinderCheck = solidCheck->asCylinder();
        ASSERT_EQUAL_DOUBLE(cylinder->radius, cylinderCheck->radius);
        ASSERT_EQUAL_DOUBLE(cylinder->length, cylinderCheck->length);

    } else if (solid->isSphere()) {
        const Sphere* sphere = solid->asSphere();
        const Sphere* sphereCheck = solidCheck->asSphere();
        ASSERT_EQUAL_DOUBLE(sphere->radius, sphereCheck->radius);

    } else if (solid->isExternalMesh()) {
        const ExternalMesh* mesh = solid->asExternalMesh();
        const ExternalMesh* meshCheck = solidCheck->asExternalMesh();
        ASSERT_EQUAL_VECTOR(mesh->scale, meshCheck->scale);
        ASSERT_IS_TRUE(mesh->filename == meshCheck->filename);
    } else {
        ASSERT_IS_TRUE(false);
    }

}


void checkImportExportURDF(std::string fileName)
{
    // Import, export and re-import a URDF file, and check that the key properties of the model are mantained
    ModelLoader mdlLoader;
    bool ok = mdlLoader.loadModelFromFile(fileName, "urdf");
    Model model = mdlLoader.model();
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model loaded from " << fileName << std::endl;
    std::string urdfString;
    ModelExporter mdlExporter;
    ok = mdlExporter.init(model, SensorsList());
    ASSERT_IS_TRUE(ok);
    ok = mdlExporter.exportModelToString(urdfString);
    ASSERT_IS_TRUE(ok);

    std::cerr << "Model serialized back to xml " << std::endl << urdfString << std::endl;

    ModelLoader mdlLoaderReloaded;

    ok = mdlLoaderReloaded.loadModelFromString(urdfString);
    ASSERT_IS_TRUE(ok);
    Model modelReloaded = mdlLoaderReloaded.model();

    ASSERT_EQUAL_DOUBLE(model.getNrOfLinks(), modelReloaded.getNrOfLinks());
    ASSERT_EQUAL_DOUBLE(model.getNrOfJoints(), modelReloaded.getNrOfJoints());
    ASSERT_EQUAL_DOUBLE(model.getNrOfDOFs(), modelReloaded.getNrOfDOFs());
    ASSERT_EQUAL_DOUBLE(model.getNrOfFrames(), modelReloaded.getNrOfFrames());
    ASSERT_EQUAL_DOUBLE(getNrOfVisuals(model), getNrOfVisuals(modelReloaded));
    ASSERT_EQUAL_DOUBLE(getNrOfCollisions(model), getNrOfCollisions(modelReloaded));

    // Verify that the link correspond (note that the serialization could have changed)
    for(int lnkIndex=0; lnkIndex < model.getNrOfLinks(); lnkIndex++) {
        LinkIndex lnkIndexInReloaded = modelReloaded.getLinkIndex(model.getLinkName(lnkIndex));
        ASSERT_IS_TRUE(lnkIndexInReloaded != LINK_INVALID_INDEX);
        ASSERT_IS_TRUE(model.getLinkName(lnkIndex) == modelReloaded.getLinkName(lnkIndexInReloaded));
        SpatialInertia inertia = model.getLink(lnkIndex)->getInertia();
        SpatialInertia inertiaReloaded = modelReloaded.getLink(lnkIndexInReloaded)->getInertia();
        std::cerr << "Testing inertia of link " << model.getLinkName(lnkIndex) << std::endl;
        ASSERT_EQUAL_MATRIX(inertia.asMatrix(), inertiaReloaded.asMatrix());

        // For each link, verify that the visual and collision shape correspond

        // First verify that the number of visual elements are the same
        ASSERT_EQUAL_DOUBLE(model.visualSolidShapes().linkSolidShapes[lnkIndex].size(),
                            modelReloaded.visualSolidShapes().linkSolidShapes[lnkIndexInReloaded].size());

        // Then, if there is only one element, verify that it matches
        if (model.visualSolidShapes().linkSolidShapes[lnkIndex].size() == 1) {
            SolidShape* solidInModel = model.visualSolidShapes().linkSolidShapes[lnkIndex][0];
            SolidShape* solidInModelReloaded = modelReloaded.visualSolidShapes().linkSolidShapes[lnkIndexInReloaded][0];
            std::cerr << "original: " << solidInModel->link_H_geometry.toString() << std::endl
                      << "reloaded: " << solidInModelReloaded->link_H_geometry.toString() << std::endl;
            checkSolidAreEqual(solidInModel, solidInModelReloaded);
        }

        // First verify that the number of visual elements are the same
        ASSERT_EQUAL_DOUBLE(model.collisionSolidShapes().linkSolidShapes[lnkIndex].size(),
                            modelReloaded.collisionSolidShapes().linkSolidShapes[lnkIndexInReloaded].size());

        // Then, if there is only one element, verify that it matches
        if (model.collisionSolidShapes().linkSolidShapes[lnkIndex].size() == 1) {
            SolidShape* solidInModel = model.collisionSolidShapes().linkSolidShapes[lnkIndex][0];
            SolidShape* solidInModelReloaded = modelReloaded.collisionSolidShapes().linkSolidShapes[lnkIndexInReloaded][0];
            checkSolidAreEqual(solidInModel, solidInModelReloaded);
        }
    }

    // Verify that the frame correspond (note that the serialization could have changed)
    for(FrameIndex frameIndex=model.getNrOfLinks(); frameIndex < model.getNrOfFrames(); frameIndex++) {
        FrameIndex frameIndexInReloaded = modelReloaded.getFrameIndex(model.getFrameName(frameIndex));
        ASSERT_IS_TRUE(frameIndexInReloaded != FRAME_INVALID_INDEX);
        ASSERT_IS_TRUE(model.getFrameName(frameIndex) == modelReloaded.getFrameName(frameIndexInReloaded));
        Transform link_H_frame = model.getFrameTransform(frameIndex);
        Transform link_H_frame_reloaded = modelReloaded.getFrameTransform(frameIndexInReloaded);
        ASSERT_EQUAL_MATRIX(link_H_frame.asHomogeneousTransform(), link_H_frame_reloaded.asHomogeneousTransform());
    }

}


int main()
{
    for(unsigned int mdl = 0; mdl < IDYNTREE_TESTS_URDFS_NR; mdl++ )
    {
        if (std::string(IDYNTREE_TESTS_URDFS[mdl]) == "bigman.urdf")
        {
            // walkman model fails this test due to https://github.com/robotology/idyntree/issues/247
            continue;
        }

        std::string urdfFileName = getAbsModelPath(std::string(IDYNTREE_TESTS_URDFS[mdl]));
        std::cout << "Checking model import/export test on " << urdfFileName << std::endl;
        checkImportExportURDF(urdfFileName);
    }


    return EXIT_SUCCESS;
}
