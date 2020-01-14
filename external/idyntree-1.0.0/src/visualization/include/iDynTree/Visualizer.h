/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 *
 * Licensed under either the GNU Lesser General Public License v3.0 :
 * https://www.gnu.org/licenses/lgpl-3.0.html
 * or the GNU Lesser General Public License v2.1 :
 * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
 * at your option.
 */

#ifndef IDYNTREE_VISUALIZER_H
#define IDYNTREE_VISUALIZER_H

#include <string>

#include <iDynTree/Core/Direction.h>
#include <iDynTree/Core/Position.h>

#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/LinkState.h>

namespace iDynTree
{
class Model;
class Transform;
class Visualizer;

/**
 * Interface to manipulate the camera parameters.
 */
class ICamera
{
public:
    /**
      * Destructor
      */
    virtual ~ICamera() = 0;

    /**
     * Set the linear position of the camera w.r.t to the world.
     */
    virtual void setPosition(const iDynTree::Position & cameraPos) = 0;

    /**
     * Set the target of the camera (i.e. the point the camera is looking into) w.r.t. the world.
     */
    virtual void setTarget(const iDynTree::Position & cameraPos) = 0;

    /**
     * Set the up vector of the camera w.r.t to the world.
     */
    virtual void setUpVector(const Direction& upVector) = 0;
};

/**
 * Basic structure to encode color information
 */
class ColorViz
{
public:
    /**
     * Red component of the color.
     */
    float r;

    /**
     * Green component of the color.
     */
    float g;

    /**
     * Blue component of the color.
     */
    float b;

    /**
     * Alpha component of the color.
     */
    float a;

    /**
     * Default constructor (to white)
     */
    ColorViz();

    /**
     * Constructor for rgba.
     */
    ColorViz(float r, float g, float b, float a);

    /**
     * Build a color from a Vector4 rgba.
     */
    ColorViz(const Vector4 & rgba);
};

enum LightType
{
    //! point light, it has a position in space and radiates light in all directions
    POINT_LIGHT,
    //! directional light, coming from a direction from an infinite distance
    DIRECTIONAL_LIGHT
};

/**
 * Interface to a light visualization.
 */
class ILight
{
public:
    /**
     * Denstructor
     */
    virtual ~ILight() = 0;

    /**
     * Get the name.
     */
    virtual const std::string & getName() const = 0;

    /**
     * Set the light type.
     */
    virtual void setType(const LightType type) = 0;

    /**
     * Get the light type.
     */
    virtual LightType getType() = 0;

    /**
     * Set the linear position of the light w.r.t to the world.
     */
    virtual void setPosition(const iDynTree::Position & cameraPos) = 0;

    /**
     * Get the linear position of the light w.r.t to the world.
     */
    virtual iDynTree::Position getPosition() = 0;

    /**
     * Set the light direction (only meaningful if the light is DIRECTIONAL_LIGHT).
     */
    virtual void setDirection(const Direction& lightDirection) = 0;

    /**
     * Get the light direction (only meaningful if the light is DIRECTIONAL_LIGHT).
     */
    virtual Direction getDirection() = 0;

    /**
     * Set ambient color of the light.
     */
    virtual void setAmbientColor(const ColorViz & ambientColor) = 0;

    /**
     * Get ambient color of the light.
     */
    virtual ColorViz getAmbientColor() = 0;

    /**
     * Set specular color of the light.
     */
    virtual void setSpecularColor(const ColorViz & ambientColor) = 0;

    /**
     * Get specular color of the light.
     */
    virtual ColorViz getSpecularColor() = 0;

    /**
     * Set ambient color of the light.
     */
    virtual void setDiffuseColor(const ColorViz & ambientColor) = 0;

    /**
     * Get ambient color of the light.
     */
    virtual ColorViz getDiffuseColor() = 0;
};

/**
 * Interface to manipulate the elements in the enviroment (background, root frame, reference lines)
 */
class IEnvironment
{
public:
    /**
     * Denstructor
     */
    virtual ~IEnvironment() = 0;

    /**
     * Get the list of the elements in the enviroment.
     *
     * The function returns the following list:
     *
     * | Element name  | Description |
     * |:-------------:|:-----------:|
     * |  floor_grid   | Grid used to indicated the z = 0 plane. |
     * |  world_frame  | XYZ (RBG) arrows indicating the world frame. |
     */
    virtual std::vector<std::string> getElements() = 0;

    /**
     *
     * @return true if the visibility is correctly setted, false otherwise.
     */
    virtual bool setElementVisibility(const std::string elementKey, bool isVisible) = 0;

    /**
     * Set the background color.
     */
    virtual void setBackgroundColor(const ColorViz & backgroundColor) = 0;

    /**
     * Set the ambient light of the enviroment.
     */
    virtual void setAmbientLight(const ColorViz & ambientLight) = 0;

    /**
     * Get the list of lights present in the visualization.
     */
    virtual std::vector<std::string> getLights() = 0;

    /**
     * Add a light.
     */
    virtual bool addLight(const std::string & lightName) = 0;

    /**
     * Return an interface to a light.
     */
    virtual ILight & lightViz(const std::string & lightName) = 0;

    /**
     * Remove a light from visualization.
     *
     * @return true if the light was present and was removed, false otherwise.
     */
    virtual bool removeLight(const std::string & lightName) = 0;
};

/**
 * Interface to the visualization of jets attached to a model.
 */
class IJetsVisualization
{
public:
    /**
     * Denstructor
     */
    virtual ~IJetsVisualization() = 0;

    /**
     * Set the frame on the model on which the jets are visualized.
     *
     * @note this will delete any state related to jets visualization.
     */
    virtual bool setJetsFrames(const std::vector< std::string > & jetsFrames) = 0;

    /**
     * Get the number of visualized jets.
     *
     */
    virtual size_t getNrOfJets() const = 0;

    /**
     * Get jet direction.
     */
    virtual Direction getJetDirection(const int jetIndex) const = 0;

    /**
     * Set jet direction.
     */
    virtual bool setJetDirection(const int jetIndex, const Direction & jetDirection) = 0;

    /**
     * Set jet color.
     */
    virtual bool setJetColor(const int jetIndex, const ColorViz & jetColor) = 0;

    /**
     * The jets are visualized as cones attached to the frame,
     */
    virtual bool setJetsDimensions(const double & minRadius, const double & maxRadius, const double & maxLenght) = 0;

    /**
     * Set the jets intensity.
     *
     * @param[in] jetsIntensity a vector of getNrOfJets values, from 0 (no thrust) 1 (max thrust).
     *
     */
    virtual bool setJetsIntensity(const VectorDynSize & jetsIntensity) = 0;
};

/**
 * Interface to the visualization of vectors.
 */
class IVectorsVisualization
{
public:
    /**
     * Denstructor
     */
    virtual ~IVectorsVisualization() = 0;

    /**
     * @brief Add a vector in the visualization
     * @return The vector index.
     */
    virtual size_t addVector(const Position & origin, const Direction & direction, double modulus) = 0;

    /**
     * @brief Add a vector in the visualization
     * @return The vector index.
     */
    virtual size_t addVector(const Position & origin, const Vector3 & components) = 0;

    /**
     * Get the number of visualized vectors.
     *
     */
    virtual size_t getNrOfVectors() const = 0;

    /**
     * Get vector properties.
     */
    virtual bool getVector(size_t vectorIndex, Position & currentOrigin,
                           Direction & currentDirection, double & currentModulus) const = 0;

    /**
     * Get vector properties.
     */
    virtual bool getVector(size_t vectorIndex, Position & currentOrigin, Vector3 & components) const = 0;

    /**
     * Update Vector
     */
    virtual bool updateVector(size_t vectorIndex, const Position & origin, const Direction & direction, double modulus) = 0;

    /**
     * Update Vector
     */
    virtual bool updateVector(size_t vectorIndex, const Position & origin, const Vector3& components) = 0;

    /**
     * Set vector color.
     */
    virtual bool setVectorColor(size_t vectorIndex, const ColorViz & vectorColor) = 0;

    /**
     * @brief Determines the dimension of the visualized arrows
     * @param zeroModulusRadius Constant offset for the arrow radius.
     * @param modulusMultiplier Multiplies the modulus and adds up to the zeroModulusRadius to get the total arrow radius.
     * @return true if successfull, false in case of negative numbers.
     */
    virtual bool setVectorsAspect(double zeroModulusRadius, double modulusMultiplier, double heightScale) = 0;
};


/**
 * Interface to the visualization of a model istance.
 */
class IModelVisualization
{
public:
    /**
     * Denstructor
     */
    virtual ~IModelVisualization() = 0;

    /**
     * Set the position of the model (using base position and joint positions)
     */
    virtual bool setPositions(const Transform & world_H_base, const VectorDynSize & jointPos) = 0;

    /**
     * Set the positions of the model by directly specifing link positions wrt to the world.
     */
    virtual bool setLinkPositions(const LinkPositions & linkPos) = 0;

    /**
     * Reference to the used model.
     */
    virtual Model & model() = 0;

    /**
     * Get the instance name.
     */
    virtual std::string getInstanceName() = 0;

    /**
     * Set the visibility of all the link of the model.
     */
    virtual void setModelVisibility(const bool isVisible) = 0;

    /**
     * Set the color of all the geometries of the model.
     *
     * This will overwrite the material of the model, but it can be
     * reset by resetModelColor.
     */
    virtual void setModelColor(const ColorViz & modelColor) = 0;

    /**
     * Reset the colors of the model.
     */
    virtual void resetModelColor() = 0;    
    
    /**
     * Set the color of all the geometries of the given link.
     *
     * This will overwrite the material of the link, but it can be
     * reset by resetLinkColor.
     */
    virtual bool setLinkColor(const LinkIndex& linkIndex, const ColorViz& linkColor) = 0;
    
    /**
     * Reset the colors of given link.
     */
    virtual bool resetLinkColor(const LinkIndex& linkIndex) = 0;
    
    /**
     * Get the name of the link in the model.
     */
    virtual std::vector< std::string > getLinkNames() = 0;

    /**
     * Set a given link visibility.
     */
    virtual bool setLinkVisibility(const std::string & linkName, bool isVisible) = 0;

    /**
     * Get list of visualization features that can be enabled/disabled.
     *
     * This method will return the follow list:
     * | Visualization  feature | Description                              | Default value |
     * |:----------------------:|:----------------------------------------:|:-------------:|
     * |  wireframe             | Visualize mesh of the model as wireframe | false  |
     */
    virtual std::vector<std::string> getFeatures() = 0;

    /**
     * @return true if the visibility is correctly setted, false otherwise.
     */
    virtual bool setFeatureVisibility(const std::string& elementKey, bool isVisible) = 0;

    /**
     * Get a reference to the internal IJetsVisualization interface.
     */
    virtual IJetsVisualization& jets() = 0;
        
    /**
     * Get the transformation of the model (root link) with respect to visualizer world \f$ w_H_{root}\f$
     * The obtained transformation matrix can be used to map any homogeneous vector from the
     * model's root link frame to the visualizer world frame.
     */
    virtual Transform getWorldModelTransform() = 0;
    
    /**
     * Get the transformation of given link with respect to visualizer world \f$ w_H_{link}\f$
     * The obtained transformation matrix can be used to map any homogeneous vector from the
     * given link frame to the visualizer world frame.
     */
    virtual Transform getWorldLinkTransform(const LinkIndex& linkIndex) = 0;
};

/**
 * Visualizer options
 */
struct VisualizerOptions
{
    /**
     * Set the visualizer to be verbose, useful for debug (default : false).
     */
    bool verbose;

    /**
     * Initial width (in pixels) of the created windows (default: 800).
     */
    int winWidth;

    /**
     * Initial height (in pixels) of the created window (default: 600).
     */
    int winHeight;

    /**
     * Dimension of the root frame arrows in meters (default: 1.0).
     */
    double rootFrameArrowsDimension;

    VisualizerOptions(): verbose(false),
                         winWidth(800),
                         winHeight(600),
                         rootFrameArrowsDimension(1.0)
    {
    }
};

/**
 * Class to visualize a set of iDynTree models
 */
class Visualizer
{
friend class ModelVisualization;

private:
    struct VisualizerPimpl;
    VisualizerPimpl * pimpl;

    // Disable copy for now
    Visualizer(const Visualizer& other);
    Visualizer& operator=(const Visualizer& other);
public:
    Visualizer();
    virtual ~Visualizer();

    /**
     * Initialize the visualization.
     *
     * \note this is called implicitly when addModel is called for the first time.
     */
    bool init(const VisualizerOptions = VisualizerOptions());

    /**
     * Get number of models visualized.
     */
    size_t getNrOfVisualizedModels();

    /**
     *
     */
    std::string getModelInstanceName(size_t modelInstanceIndex);

    /**
     * Get the index of a given model instance, or -1 if there is not such model instance.
     */
    int getModelInstanceIndex(const std::string instanceName);

    /**
     * Add an instance of a given model to the visualization.
     *
     * @param[in] model
     * @param[in] instanceName name of the instance of the model added.
     * @return true if all went well, false otherwise.
     */
    bool addModel(const iDynTree::Model & model,
                  const std::string & instanceName);

    /**
     * Return an interface to a visualization of a model.
     *
     * \note the modelIdx is invalidated whenever a model is removed from the visualization.
     *
     * @return a reference to a valid ModelVisualization if instanceName is the name of a model instance.
     */
    IModelVisualization& modelViz(size_t modelIdx);

    /**
     * Return an interface to a visualization of a model.
     *
     * @return a reference to a valid ModelVisualization if instanceName is the name of a model instance.
     */
    IModelVisualization& modelViz(const std::string & instanceName);

    /**
     * Return an interface to manipulate the camera in the visualization.
     */
    ICamera& camera();

    /**
     * Return an interface to manipulate the visualization environment.
     */
    IEnvironment& enviroment();

    /**
     * Get a reference to the internal IVectorsVisualization interface.
     */
    IVectorsVisualization& vectors();

    /**
     * Wrap the run method of the Irrlicht device.
     */
    bool run();

    /**
     * Draw the visualization.
     */
    void draw();

    /**
     * Right the current visualization to a image file.
     *
     * The format of the image is desumed from the filename.
     *
     * For more info on the process of writing the image,
     * check irr::video::IVideoDriver::writeImageToFile irrlicht method.
     *
     * @return true if all went ok, false otherwise.
     */
    bool drawToFile(const std::string filename="iDynTreeVisualizerScreenshot.png");

    /**
     * Close the visualizer.
     */
    void close();
};

}

#endif
