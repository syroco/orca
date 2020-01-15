/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef THREE_AXIS_ANGULAR_ACCELEROMETER_H
#define THREE_AXIS_ANGULAR_ACCELEROMETER_H


namespace iDynTree
{
    class Transform;
    class LinearMotionVector3;
    typedef LinearMotionVector3 LinAcceleration;
    class SpatialAcc;
    class Twist;
}

#include <iDynTree/Sensors/Sensors.h>

namespace iDynTree {

    /**
     * Class representing a three axis angular accelerometer, i.e. a sensor that measures
     * the 3D angular acceleration of the sensor frame.
     *
     * \ingroup iDynTreeSensors
     *
     */
    class ThreeAxisAngularAccelerometerSensor: public LinkSensor
    {
    private:
        struct ThreeAxisAngularAccelerometerPrivateAttributes;
        ThreeAxisAngularAccelerometerPrivateAttributes * pimpl;

    public:
        /**
         * Constructor.
         */
        ThreeAxisAngularAccelerometerSensor();

        /**
         * Copy constructor
         */
        ThreeAxisAngularAccelerometerSensor(const ThreeAxisAngularAccelerometerSensor& other);

        /**
         * Copy operator
         */
        ThreeAxisAngularAccelerometerSensor& operator=(const ThreeAxisAngularAccelerometerSensor &other);

        /**
         * Destructor.
         */
        virtual ~ThreeAxisAngularAccelerometerSensor();

        /**
         * Set the name (id) of the sensor
         *
         */
        bool setName(const std::string &_name);

       /**
         * Set the transform from the sensor to the parent link attached to the sensor.
         *
         * @return true if link_index is parent link attached to the accelerometer sensor, false otherwise.
         */
        bool setLinkSensorTransform(const iDynTree::Transform & link_H_sensor);

        /*
         * Documented in Sensor
         */
        bool setParentLink(const std::string &parent);

        /*
         * Documented in Sensor
         */
        bool setParentLinkIndex(const LinkIndex & parent_index);

        /*
         * Documented in the sensor
         *
         */
        std::string getName() const;

        /*
         * Documented in Sensor
         */
        SensorType getSensorType() const;


        /*
         * Documented in Sensor
         */
        std::string getParentLink() const;

        /*
         * Documented in Sensor
         */
        LinkIndex getParentLinkIndex() const;

        // Documented in LinkSensor
        Transform getLinkSensorTransform() const;

        /*
         * Documented in Sensor
         */
        bool isValid() const;

        /*
         * Documented in Sensor
         */
        Sensor * clone() const;

        /*
         * Documented in Sensor
         */
        bool updateIndices(const Model & model);

       /**
        * Simulate the measurement of the Three Axis angular accelerometer
        *
        * @param[in] linkAcc the left trivialized acceleration of the link frame w.r.t. to an inertial frame
        *
        * @return the predicted measurement as a AngAcceleration
        */
        Vector3 predictMeasurement(const iDynTree::SpatialAcc &linkAcc);

        // Deprecated
        IDYNTREE_DEPRECATED_WITH_MSG("Use predictMeasurement(const iDynTree::SpatialAcc &linkAcc) instead")
        Vector3 predictMeasurement(const iDynTree::SpatialAcc &linkAcc, const iDynTree::Twist &linkTwist);
    };





}



#endif
