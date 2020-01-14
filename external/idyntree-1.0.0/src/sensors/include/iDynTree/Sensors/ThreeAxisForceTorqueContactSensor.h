/*
 * Copyright (C) 2017 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */


#ifndef THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR_H
#define THREE_AXIS_FORCE_TORQUE_CONTACT_SENSOR_H


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
     * Class representing a three axis force-torque contact sensor.
     *
     * It is meant to represent sole sensor composed of a array of uniaxial
     * pressure sensors, such as the one present in the soles of Boston Dynamics
     * Atlas, SoftBank Robotics Nao or ROBOTIS OP2.
     *
     * The sensor is assumed to be measuring the z-component of the force and the
     * x,y component of the torque of the net external force/torque applied on
     * the link expressed in the sensor frame.
     *
     * \note While the SixAxisFTSensor is meant to model the force torque measured
     * at a joint, this sensor is modeled as measuring directly (part) of the external
     * contact force exerted on a link.
     *
     * \ingroup iDynTreeSensors
     *
     */
    class ThreeAxisForceTorqueContactSensor: public LinkSensor
    {
    private:
        struct ThreeAxisForceTorqueContactSensorPrivateAttributes;
        ThreeAxisForceTorqueContactSensorPrivateAttributes * pimpl;

    public:
        /**
         * Constructor.
         */
        ThreeAxisForceTorqueContactSensor();

        /**
         * Copy constructor
         */
        ThreeAxisForceTorqueContactSensor(const ThreeAxisForceTorqueContactSensor& other);

        /**
         * Copy operator
         */
        ThreeAxisForceTorqueContactSensor& operator=(const ThreeAxisForceTorqueContactSensor &other);

        /**
         * Destructor.
         */
        virtual ~ThreeAxisForceTorqueContactSensor();

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
         * Helper methods to conver from raw load  cells measurements (expressed in Newtons)
         * to the measurement of the three axis F/T sensors.
         */
        ///@{

        /**
         * Set the load cells locations in sensor frame.
         *
         * Note the load cells are assumed to be measured the
         * force applied on the link, on the z-direction in the sensor
         * frame, coherently with the sensor definition.
         */
        void setLoadCellLocations(std::vector<Position> &loadCellLocations);

        /**
         * Get the load cells locations in sensor frame.
         */
        std::vector<Position> getLoadCellLocations() const;

        /**
         * Compute the force/torque measurements from the raw load cell readings.
         *
         * @return a zero vector in case of error, the result otherwise.
         */
        Vector3 computeThreeAxisForceTorqueFromLoadCellMeasurements(VectorDynSize& loadCellMeasurements) const;

        /**
         * Compute the center of pressure in sensor frame from the raw load cell readings.
         */
        Position computeCenterOfPressureFromLoadCellMeasurements(VectorDynSize& loadCellMeasurements) const;

        ///@}

    };





}



#endif
