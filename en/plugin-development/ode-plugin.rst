Description of the ODEPlugin
============================

As explained in the :doc:`hello-world-sample`  and the :doc:`sample1` sections, it is possible to incorporate a user-developed program in Choreonoid using the Plugin format. Here, we will explain how to incorporate a physics engine, which is at the core of the simulation. In the explanation, we will use the ODEPlugin, which is the plugin derived from the common ODE (Open Dynamics Engine) physics engine. The source file is at src/ODEPlugin. Refer to  `http://www.ode.org/ <http://www.ode.org/>`_ for details about ODE.

.. contents:: 目次
   :local:


ODE plugin
----------

.. highlight:: cpp

First, the source code for the plugin’s basic structure. (Since the sample source file is shared with the code for GAZEBO, some parts differ from the following contents. Also, this explanation includes features that we do not describe, so those parts are omitted.) ::

 #include "ODESimulatorItem.h"
 #include <cnoid/Plugin>
 #include <ode/ode.h>
 #define PLUGIN_NAME "ODE"

 class ODEPlugin : public Plugin
 {
 public:
     ODEPlugin() : Plugin(PLUGIN_NAME)
     {
         require("Body");
     }
     virtual bool initialize()
     {
        ............
     }

     virtual bool finalize()
     {
        ...............
     }
 };
 CNOID_IMPLEMENT_PLUGIN_ENTRY(ODEPlugin);

We are handling a robot model, so the constructor displays that the Body plugin is required. Also, we include the ODE header files.

Plugin initialization
~~~~~~~~~~~~~~~~~~~~~

After the plugin is read into memory, this initialization function is executed only once ::

 virtual bool initialize()
 {
     dInitODE2(0);
     dAllocateODEDataForThread(dAllocateMaskAll);

     ODESimulatorItem::initializeClass(this);
             
     return true;
 }

The ODE initialization function is being called. (The function name starts with d, so it is an ODE function.) ODESimulatorItem will be explained in detail later. Here, we call that initialization function.

Closing the plugin
~~~~~~~~~~~~~~~~~~

This function is executed only once when the plugin is destroyed (when Choreonoid quits). ::

 virtual bool finalize()
 {
     dCloseODE();
     return true;
 }

The ODE closing function is being called. It has now been formed as a plugin.

ODE simulator item
------------------

The physics engine is implemented as a  :ref:`simulation_simulator_item`  so that Choreonoid can handled it as an item. The ODEPlugin implements the ODESimulatorItem for using ODE with Choreonoid.

ODESimulatorItem is defined as a class inherited from SimulatorItem. The header file performing this is shown below. : ::

 #include <cnoid/SimulatorItem>
 #include "exportdecl.h"

 namespace cnoid {
         
 class CNOID_EXPORT ODESimulatorItem : public SimulatorItem
 {
 public:
     static void initializeClass(ExtensionManager* ext);
    ..........................
 };
 }

It is a static initialization function called when the plugin is initialized. It registers the ODESimulatorItem in itemManager, which manages items, and allows the ODESimulatorItem to be made from the menu. ::

 void ODESimulatorItem::initializeClass(ExtensionManager* ext)
 {
     ext->itemManager().registerClass<ODESimulatorItem>(ITEM_NAME);
     ext->itemManager().addCreationPanel<ODESimulatorItem>();
 }

When the ODE simulator item is added to an item, an ODESimulatorItem class object is created. In the constructor, you set the initial values of user-changeable parameters and initialize the variables. ::

 ODESimulatorItem::ODESimulatorItem()
 {
     initialize();
     stepMode.setSymbol(ODESimulatorItem::STEP_ITERATIVE,  N_("Iterative (quick step)"));
     gravity << 0.0, 0.0, -DEFAULT_GRAVITY_ACCELERATION;
     .............
 }

The doDuplicate function is called when a new ODE simulator item is created. Implement it so that a new object is created and the pointer is returned. ::

 ItemPtr ODESimulatorItem::doDuplicate() const
 {
     return new ODESimulatorItem(*this);
 }

When the ODE simulator item is deleted from the GUI, an ODESimulatorItem class object is also destroyed. In the destructor, release the amount of memory, etc. as required. ::

 ODESimulatorItem::~ODESimulatorItem()
 {
     clear();
     if(contactJointGroupID){
         dJointGroupDestroy(contactJointGroupID);
     }
 }

This function is called when displaying parameters in the Property view or when changing parameter values.

.. code-block:: cpp

    void ODESimulatorItem::doPutProperties(PutPropertyFunction& putProperty)
    {
        SimulatorItem::doPutProperties(putProperty);
        // We are setting properties that are common to simulator items, so be sure to call them.
     
        putProperty(_("Step mode"), stepMode, changeProperty(stepMode));
        // This function configures the parameter settings. Set the parameter name, variables, and called functions.
    }

This function saves the parameter settings in the project file.

.. code-block:: cpp

    bool ODESimulatorItem::store(Archive& archive)
    {
        SimulatorItem::store(archive);
        // We are saving properties that are common to simulator items, so be sure to call them.
    
        archive.write("stepMode", stepMode.selectedSymbol());
        // Set the saved parameter name and variables.
    
        write(archive, "gravity", gravity);
        // Vector-type variables use this function.
    }

This function reads the parameter settings from the project file.

.. code-block:: cpp

    bool ODESimulatorItem::restore(const Archive& archive)
    {
        SimulatorItem::restore(archive);
        // We are reading properties that are common to simulator items, so be sure to call them.

        archive.read("friction", friction);
        // Set the read parameter name and variables.

        read(archive, "gravity", gravity);
        // Vector-type variables use this function.
    }

Execution of simulation
~~~~~~~~~~~~~~~~~~~~~~~

Next, the execution of the central part of the simulation. First, we will explain the overall process.

When the user clicks the button to start the simulation, first the createSimulationBody function which creates the ODE models is called the same number of times as the number of target models to be simulated.

Many physics engines have their own original method of describing models. ODE is no different. In Choreonoid, robots and environments are kept as Body objects. It is necessary to construct ODE models from these Body objects.

The argument orgBody contains a pointer to the Body object, so we will now create an ODEBody object for ODE and return that pointer. At this point, an actual ODE model has not yet been created. ::

 SimulationBodyPtr ODESimulatorItem::createSimulationBody(BodyPtr orgBody)
 {
     return new ODEBody(*orgBody);
 }

The ODEBody class is created by inheriting from the SimulationBody class. ::

 class ODEBody : public SimulationBody
 {
 public:
     ..................
 }
 
 ODEBody::ODEBody(const Body& orgBody)
     : SimulationBody(new Body(orgBody))
 {
    worldID = 0;
    ...............
 }

next, the initialization function will be called just once. The simBodies argument contains a pointer to the ODEBody object created above for the simulation.

.. code-block:: cpp

    bool ODESimulatorItem::initializeSimulation(const std::vector<SimulationBody*>& simBodies)
    {
         clear();
         // The result of the previous simulation is destroyed.
    
         dRandSetSeed(0);
         dWorldSetGravity(worldID, g.x(), g.y(), g.z());
         dWorldSetERP(worldID, globalERP);
         .............
         // We set the parameters for the simulation.

         timeStep = self->worldTimeStep();
         //  We can get the simulation time increment using worldTimeStep().

         for(size_t i=0; i < simBodies.size(); ++i){
             addBody(static_cast<ODEBody*>(simBodies[i]));
         }
         // We create the ODE model in the simulation world.
         // The models are added by calling addBody the same number of times as the number of target models.

         return true;
     }

After that, a function which moves the simulation ahead by one step is called repeatedly until the simulation reaches the end. The activeSimBodies argument contains a pointer to the ODEBody object created for the simulation.

.. code-block:: cpp
    
    bool ODESimulatorItem::stepSimulation(const std::vector<SimulationBody*>& activeSimBodies)
    {
        for(size_t i=0; i < activeSimBodies.size(); ++i){
            ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);
            odeBody->body()->setVirtualJointForces();
            // This calls the BodyCustomizer function.

            odeBody->setTorqueToODE();
            // This sets the joint torque for each ODEBody object.
        }
    
        dJointGroupEmpty(contactJointGroupID);
        dSpaceCollide(spaceID, (void*)this, &nearCallback);
        // This performs collision detection.

        if(stepMode.is(ODESimulatorItem::STEP_ITERATIVE)){
            dWorldQuickStep(worldID, timeStep);
        } else {
            dWorldStep(worldID, timeStep);
        }
        // This moves the simulation time ahead by one step.

        for(size_t i=0; i < activeSimBodies.size(); ++i){
            ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);

            if(!odeBody->sensorHelper.forceSensors().empty()){
                odeBody->updateForceSensors(flipYZ);
            }
            odeBody->getKinematicStateFromODE(flipYZ);
            if(odeBody->sensorHelper.hasGyroOrAccelSensors()){
                odeBody->sensorHelper.updateGyroAndAccelSensors();
            }
        }
        // This reads the result of moving ahead by one step. from each ODEBody object.

        return true;
    }

.. note:: The code odeBody->body()->setVirtualJointForces() is above. This is a mechanism called BodyCustomizer, which allows you to dynamically incorporate model-specific programs into the dynamics calculation library. The project for this sample is CustomizedSpringModel.cnoid. The sample program is at sample/SpringModel/SpringModelCustomizer.cpp. Refer to the `Method of spring damper modeling of a joint <http://www.openrtp.jp/openhrp3/jp/springJoint.html>`_ section of the OpenHRP3 website for an explanation of this sample.

Body class and Link class
~~~~~~~~~~~~~~~~~~~~~~~~~

Next, before explaining the structure of the ODE model, we will explain the Body class and the Link class for describing physical objects within Choreonoid. (For details on how to describe the VRML model, refer to the  `Robot and Environmental Model Define Format <http://www.openrtp.jp/openhrp3/jp/create_model.html>`_ section of the OpenHRP3 website.)

The Body object manages Link objects in the form of a tree structure. An environmental model like a floor is also a Body object consisting of one Link object. The Body object will always have a root link, which forms the root of the tree structure.

The Body class provides the following functions:

.. list-table:: Body class functions
   :widths: 30 60
   :header-rows: 1

   * - Function
     - Feature
   * - int numJoints()
     - Returns the total number of joints.
   * - Link* joint(int id)
     - Returns a pointer to the Link object corresponding to a joint id.
   * - int numLinks()
     - Returns the total number of links.
   * - Link* link(int index)
     - Returns a pointer to the Link object corresponding to a link id.
   * - Link* link(const std::string& name)
     - Returns a pointer to a Link object with a matching link name.
   * - Link* rootLink()
     - Returns a pointer to the root link.
   * - int numDevices()
     - Returns the total number of devices.　The Device class is a parent class for describing such things as force sensors.
   * - Device* device(int index)
     - Returns a Device object corresponding to a device id.
   * - template<class DeviceType> DeviceList<DeviceType> devices()
     - | Returns a list of devices.
       | For example, to get a device list of force sensors, it would look like this:
       | DeviceList<ForceSensor> forceSensors = body->devices();
   * - template<class DeviceType> DeviceType* findDevice(const std::string& name)
     - Returns a pointer to a Device object with a matching name.
   * - void initializeDeviceStates()
     - Sets all devices to their initial state.
   * - bool isStaticModel()
     - Returns true when an object is static, like a floor or a wall.
   * - bool isFixedRootModel()
     - Returns true when the root link is a fixed joint.
   * - double mass()
     - Returns the total mass.
   * - const Vector3& centerOfMass() const;
     - Returns the vector of the center of mass.
   * - void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false)
     - | Calculates forward kinematics (position and orientation of the links other than the root link from the position and orientation of the root link and the angles of all joints).
       | Set calcvelocity and calcAcceleration to true and it will use the joint angle velocity and angular acceleration to calculate the link velocity and acceleration.
   * - void clearExternalForces()
     - Sets external forces to 0.
   * - numExtraJoints()
     - Returns the total number of virtual joints.
   * - ExtraJoint& extraJoint(int index)
     - Returns the virtual joint corresponding to a virtual joint id.

The Link class provides the following functions:

.. list-table:: Link class functions
   :widths: 30 60
   :header-rows: 1

   * - Function
     - Feature
   * - Link* parent()
     - Returns a pointer to the parent link.
   * - Link* sibling()
     - Returns a pointer to sibling links.
   * - Link* child()
     - Returns a pointer to child links.
   * - bool isRoot()
     - Returns true if it is a root link.
   * - | Position& T()
       | Position& position()
     - Returns a reference to the position and orientation matrix of the link origin as seen from world coordinates.
   * - Position::TranslationPart p()
     - Returns a reference to the positional vector of the link origin as seen from world coordinates.
   * - Position::LinearPart R()
     - Returns a reference to the orientation matrix of the link origin as seen from world coordinates.
   * - Position::ConstTranslationPart b()
     - Returns the positional vector of the link origin as seen from parent link coordinates.
   * - int jointId()
     - Returns the joint id.
   * - JointType jointType()
     - Returns the joint type. The types are rotational, sliding, free, fixed, (crawler/track).
   * - bool isFixedJoint()
     - Returns true when it is a fixed joint.
   * - bool isFreeJoint()
     - Returns true when it is a free joint.
   * - bool isRotationalJoint()
     - Returns true when it is a rotational joint.
   * - bool isSlideJoint()
     - Returns true when it is a sliding joint.
   * - | const Vector3& a()
       | const Vector3& jointAxis()
     - Returns the rotation axis vector of a rotational joint.
   * - const Vector3& d()
     - Returns the sliding direction vector of a sliding joint.
   * - double& q()
     - Returns a reference to the joint angle.
   * - double& dq()
     - Returns a reference to the joint’s angular velocity.
   * - double& ddq()
     - Returns a reference to the joint’s angular acceleration.
   * - double& u()
     - Returns a reference to the joint torque.
   * - const double& q_upper()
     - Returns a reference to the upper limit of the joint’s motion angle.
   * - const double& q_lower()
     - Returns a reference to the lower limit of the joint’s motion angle.
   * - Vector3& v()
     - Returns a reference to the velocity vector of the link origin as seen from world coordinates.
   * - Vector3& w()
     - Returns a reference to the angular velocity vector of the link origin as seen from world coordinates.
   * - Vector3& dv()
     - Returns a reference to the acceleration vector of the link origin as seen from world coordinates.
   * - Vector3& dw()
     - Returns a reference to the angular acceleration vector of the link origin as seen from world coordinates.
   * - | const Vector3& c()
       | const Vector3& centerOfMass()
     - Returns a reference to the center of gravity vector as seen from link’s own coordinates.
   * - | const Vector3& wc()
       | const Vector3& centerOfMassGlobal()
     - Returns a reference to the center of gravity vector as seen from world coordinates.
   * - | double m()
       | double mass()
     - Returns the mass.
   * - const Matrix3& I()
     - Returns a reference to the tensor of inertia matrix around the center of gravity as seen from link’s own coordinates.
   * - const std::string& name()
     - Returns a reference to the link name.
   * - SgNode* shape()
     - Returns a pointer to the Link shape object.
   * - Matrix3 attitude()
     - Returns the orientation matrix of the link as seen from world coordinates. (with offset)


.. note:: In Choreonoid, the local coordinate system representing the position and orientation of each link is set as follows. The coordinate origin is centered on the joint axis. The orientation when the joint angles are all 0 degrees, the orientation matrix is parallel to the world coordinate system. However, depending on the structure of the robot, it may be more convenient for the orientation of local coordinate system to have an offset. In the model description in the VRML file, the offset can be set. In Choreonoid, even if an offset is set, processing is performed to change the local coordinate system as above when reading a model file. The data obtained by the above functions is expressed in the modified coordinate system. However, the orientation matrix obtained by the attitude() function is expressed in the coordinate system before it is changed.

Structure of the ODE model
~~~~~~~~~~~~~~~~~~~~~~~~~~
Next, we will give a detailed explanation of the structure of the ODE model.

When the createSimulationBody function is called, an ODEBody object is created, but there is only a container for it and it does not yet have any substance. The actual object is created when addBody is called in initializeSimulation.

This is the addBody source code.


.. code-block:: cpp

    void ODESimulatorItemImpl::addBody(ODEBody* odeBody)
    {
         Body& body = *odeBody->body();
         // Gets a pointer to the Body object.

         Link* rootLink = body.rootLink();
         // Gets a pointer to the root link.
         rootLink->v().setZero();
         rootLink->dv().setZero();
         rootLink->w().setZero();
         rootLink->dw().setZero();
         //Sets the velocity, acceleration, angular velocity and angular acceleration of the root link to 0.
    
         for(int i=0; i < body.numJoints(); ++i){
             Link* joint = body.joint(i);
             joint->u() = 0.0;
             joint->dq() = 0.0;
             joint->ddq() = 0.0;
         }
         // The torque, angular velocity and angular acceleration of each joint are also set to 0.
         // The position and orientation of the root link and the angle of each joint are set to the initial values of the simulation.
         
         body.clearExternalForces();
         // Sets external forces to 0.
         body.calcForwardKinematics(true, true);
         // Calculates the position and orientation of each link.

         odeBody->createBody(this);
         // Creates the ODE model.
     }

This is the createBody source code.

.. code-block:: cpp

    void ODEBody::createBody(ODESimulatorItemImpl* simImpl)
    {
        Body* body = this->body();
        // Gets a pointer to the Body object.
    
        worldID = body->isStaticModel() ? 0 : simImpl->worldID;
        // Determines whether or not the model is a floor or other static object, and that handling can be changed.
    
        spaceID = dHashSpaceCreate(simImpl->spaceID);
        dSpaceSetCleanup(spaceID, 0);
        // This is preparation for ODE.

        ODELink* rootLink = new ODELink(simImpl, this, 0, Vector3::Zero(), body->rootLink());
        // Creates the model’s root link (object).
        // Constructs the whole object, from the root link to the fingers and toes.
        // The root link has no parent link, so it is given a parent link pointer of 0 and a positional vector of zero.

        setKinematicStateToODE(simImpl->flipYZ);
        // This sets the position and orientation of the ODEBody object.
        
        setExtraJoints(simImpl->flipYZ);
        // Sets the virtual joints.
       
        setTorqueToODE();
        // This sets the torque of the ODEBody object.

        sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);
        const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
        forceSensorFeedbacks.resize(forceSensors.size());
        for(size_t i=0; i < forceSensors.size(); ++i){
            dJointSetFeedback(
                odeLinks[forceSensors[i]->link()->index()]->jointID, &forceSensorFeedbacks[i]);
        }
        // Performs initialization for sensor output or force sensors, etc.
    
    }

This is the ODELink source code. The ODELink object is created from information about the Link object.

.. code-block:: cpp

    ODELink::ODELink
    (ODESimulatorItemImpl* simImpl, ODEBody* odeBody, ODELink* parent,
     const Vector3& parentOrigin, Link* link)
    {
        ...................
    
        Vector3 o = parentOrigin + link->b();
        // Calculates the positional vector of the link origin as seen from the world coordinate system.
        // parentOrigin is the positional vector of the parent link.
    
        if(odeBody->worldID){
            createLinkBody(simImpl, odeBody->worldID, parent, o);
        }
        // Sets the physical data. In ODE, static objects do not need physical data, so it is not set.
        
        createGeometry(odeBody);
        // Sets the shape data.
    
        for(Link* child = link->child(); child; child = child->sibling()){
            new ODELink(simImpl, odeBody, this, o, child);
        }
        // Follows the child links in order and creates ODELink.
    }

This is the source code for createLinkBody, which sets the ODE physical data.

.. code-block:: cpp

    void ODELink::createLinkBody
    (ODESimulatorItemImpl* simImpl, dWorldID worldID, ODELink* parent, const Vector3& origin)
    {
        bodyID = dBodyCreate(worldID);
        // Creates the ODE object (Expressed as Body in ODE. This corresponds to Link in Choreonoid)
    
        dMass mass;
        dMassSetZero(&mass);
        const Matrix3& I = link->I();
        dMassSetParameters(&mass, link->m(),
                           0.0, 0.0, 0.0,
                           I(0,0), I(1,1), I(2,2),
                           I(0,1), I(0,2), I(1,2));
        dBodySetMass(bodyID, &mass);
        // Sets the mass and tensor of inertia matrix.

        ................
    
        dBodySetRotation(bodyID, identity);
        //  Sets the orientation of the link.
        
        Vector3 p = o + c;
        dBodySetPosition(bodyID, p.x(), p.y(), p.z());
        // Sets the position of the link. ODE uses the center of gravity as the link origin.

        dBodyID parentBodyID = parent ? parent->bodyID : 0;

        switch(link->jointType()){
        // Changes the joint of the ODE to be used, depending on the type of joint.
        
            case Link::ROTATIONAL_JOINT:
            //  For a rotational joint, a hinge joint is used.
            jointID = dJointCreateHinge(worldID, 0);
            dJointAttach(jointID, bodyID, parentBodyID);
            // Connects the parent link and the link itself.
        
            dJointSetHingeAnchor(jointID, o.x(), o.y(), o.z());
            // The position of the hinge joint is the origin of the Link object.
        
            dJointSetHingeAxis(jointID, a.x(), a.y(), a.z());
            // Sets the rotation axis of the hinge joint.
            break;
        
            case Link::SLIDE_JOINT:
            //  For a sliding joint, a slider joint is used.
            jointID = dJointCreateSlider(worldID, 0);
            dJointAttach(jointID, bodyID, parentBodyID);
            // Connects the parent link and the link itself.
        
            dJointSetSliderAxis(jointID, d.x(), d.y(), d.z());
            //  Sets the slide axis of the slider joint.
            break;

            case Link::FREE_JOINT:
            // Don’t set anything for a free joint.
            break;

            case Link::FIXED_JOINT:
            default:
            // Other than the above, or for a fixed joint,
            if(parentBodyID){
                //  connect to the parent link, if there is one, with the fixed joint.
                jointID = dJointCreateFixed(worldID, 0);
                dJointAttach(jointID, bodyID, parentBodyID);
                dJointSetFixed(jointID);
                if(link->jointType() == Link::CRAWLER_JOINT){
                    simImpl->crawlerLinks.insert(make_pair(bodyID, link));
                    // Crawler (track) joints are fixed joints in ODE and treated as a special case in collision detection.
                }
            } else {
                dBodySetKinematic(bodyID);
                // If there is no parent link, set it as KinematicBody (object which does not move even if a collision occurs).
            }
            break;
        }
    }

Next is the source code for createGeometry, which sets the shape data. Shape data is described in a hierarchical structure within a Shape object.

.. code-block:: cpp
    
    void ODELink::createGeometry(ODEBody* odeBody)
    {
        if(link->shape()){
        // Gets a shape object.
        
            MeshExtractor* extractor = new MeshExtractor;
            // MeshExtractor is a utility class for moving through the hierarchy and expanding the shape data.
            
            if(extractor->extract(link->shape(), [&](){ addMesh(extractor, odeBody); })){
            // Set to move through the hierarchy and call ODELink :: addMesh every time a Mesh object is found.
            // Returning from the extract call, the triangular mesh shape has data gathered in vertices.
            
                if(!vertices.empty()){
                    triMeshDataID = dGeomTriMeshDataCreate();
                    dGeomTriMeshDataBuildSingle(triMeshDataID,
                                            &vertices[0], sizeof(Vertex), vertices.size(),
                                            &triangles[0],triangles.size() * 3, sizeof(Triangle));
                    // Converts to ODE data format.
                    
                    dGeomID gId = dCreateTriMesh(odeBody->spaceID, triMeshDataID, 0, 0, 0);
                    // Creates the ODE triangular mesh object.
                    geomID.push_back(gId);
                    dGeomSetBody(gId, bodyID);
                    //Associates it with the body of ODE.
                }
            }
            delete extractor;
        }
    }

In Choreonoid, when loading a model, all the shape data is converted to a triangular mesh shape. But if the original shape is a primitive type, that information is also saved. In the following code, the primitive types that ODE can support are used as is, and the types that cannot are created as a triangular mesh type.

This is the addMesh source code.

.. code-block:: cpp

    void ODELink::addMesh(MeshExtractor* extractor, ODEBody* odeBody)
    {
        SgMesh* mesh = extractor->currentMesh();
        // Gets a pointer to the Mesh object.

        const Affine3& T = extractor->currentTransform();
        // You can get the Mesh object’s positional and orientation matrix.

        bool meshAdded = false;

        if(mesh->primitiveType() != SgMesh::MESH){
            // You can get the shape data type using mesh->primitiveType().
            // The options are MESH, BOX, SPHERE, CYLINDER and CONE.
            // Here is the process when the shape data is a primitive type:

            bool doAddPrimitive = false;
            Vector3 scale;
            optional<Vector3> translation;
            if(!extractor->isCurrentScaled()){
            // Returns true if there have been changes to the scale.
                scale.setOnes();
                doAddPrimitive = true;
                // When there is no change in scale, each element of the scale vector is set to 1 and it is handled as a primitive type.
            } else {
                // This is the process when there is a scale change.

                Affine3 S = extractor->currentTransformWithoutScaling().inverse() *
                    extractor->currentTransform();
                // With currentTransformWithoutScaling (), you can get a coordinate transformation matrix that does not include a scale transformation matrix.
                // Extract only the scale transformation matrix.
                
                if(S.linear().isDiagonal()){
                    // Only process when the scale conversion matrix is a diagonal matrix.
                    // Otherwise, it cannot be handled in ODE as a primitive type.

                    if(!S.translation().isZero()){
                        translation = S.translation();
                        // Saves the position if there is position conversion in the scale matrix.
                    }
                    scale = S.linear().diagonal();
                    // Assigns the diagonal elements to scale.

                    if(mesh->primitiveType() == SgMesh::BOX){
                        // If the primitive type is a Box, handle it as a primitive type.
                        doAddPrimitive = true;
                    } else if(mesh->primitiveType() == SgMesh::SPHERE){
                        if(scale.x() == scale.y() && scale.x() == scale.z()){
                            //  If the primitive type is Sphere and the scale elements have the same value,
                        handle it as a primitive type.
                            doAddPrimitive = true;
                        }
                        // It cannot be handled as a primitive type if the scale elements do not have the same value.
                    } else if(mesh->primitiveType() == SgMesh::CYLINDER){
                        if(scale.x() == scale.z()){
                            // If the primitive type is Cylinder and the scale x and z elements have the same value,handle it as a primitive type.
                            doAddPrimitive = true;
                        }
                        // It cannot be handled as a primitive type if the scale x and z elements do not have the same value.
                    }
                }
            }
            if(doAddPrimitive){
                //  The process when handling as a primitive type. 
                // Creates the ODE primitive object.

                bool created = false;
                dGeomID geomId;
                switch(mesh->primitiveType()){
                case SgMesh::BOX : {
                    const Vector3& s = mesh->primitive<SgMesh::Box>().size;
                    // You can get the Box size.
                    geomId = dCreateBox(
                        odeBody->spaceID, s.x() * scale.x(), s.y() * scale.y(), s.z() * scale.z());
                    created = true;
                    break; }
                case SgMesh::SPHERE : {
                    SgMesh::Sphere sphere = mesh->primitive<SgMesh::Sphere>();
                    // You can get the Sphere radius.
                    geomId = dCreateSphere(odeBody->spaceID, sphere.radius * scale.x());
                    created = true;
                    break; }
                case SgMesh::CYLINDER : {
                    SgMesh::Cylinder cylinder = mesh->primitive<SgMesh::Cylinder>();
                    // You can get the Cylinder’s parameters.
                    geomId = dCreateCylinder(
                        odeBody->spaceID, cylinder.radius * scale.x(), cylinder.height * scale.y());
                    created = true;
                    break; }
                default :
                    break;
                }
                if(created){
                    geomID.push_back(geomId);
                    dGeomSetBody(geomId, bodyID);
                    // Ties the ODE primitive object to the ODE Body.
                
                    Affine3 T_ = extractor->currentTransformWithoutScaling();
                    // Gets the transformation matrix with the scaling removed.
                
                    if(translation){
                        T_ *= Translation3(*translation);
                        // Applies the positional transformation contained in the scale matrix.
                    }
                    Vector3 p = T_.translation()-link->c();
                    // Since the link origin is the center of gravity in ODE, we compensate for that.
                
                    dMatrix3 R = { T_(0,0), T_(0,1), T_(0,2), 0.0,
                                   T_(1,0), T_(1,1), T_(1,2), 0.0,
                                   T_(2,0), T_(2,1), T_(2,2), 0.0 };
                    if(bodyID){
                        dGeomSetOffsetPosition(geomId, p.x(), p.y(), p.z());
                        dGeomSetOffsetRotation(geomId, R);
                        // Sets the position and orientation of the shape data.
                    }else{
                        // For a static object, associate the id with the positional and orientation matrix.
                        offsetMap.insert(OffsetMap::value_type(geomId,T_));
                    }
                    meshAdded = true;
                }
            }
        }

        if(!meshAdded){
            // This is the process when it is not originally a primitive type, or it cannot be handled as a primitive type.

            const int vertexIndexTop = vertices.size();
            // Gets the number of vertex coordinates already added.

            const SgVertexArray& vertices_ = *mesh->vertices();
            //Gets a reference to the vertex coordinates within the Mesh object.
        
            const int numVertices = vertices_.size();
            for(int i=0; i < numVertices; ++i){
                const Vector3 v = T * vertices_[i].cast<Position::Scalar>() - link->c();
                // Converts the vertex vector to coordinates.
                vertices.push_back(Vertex(v.x(), v.y(), v.z()));
                // Adds to the vertex coordinates vertices in the ODELink object.
            }

            const int numTriangles = mesh->numTriangles();
            // Gets a reference to the total number of triangles within the Mesh object.
            for(int i=0; i < numTriangles; ++i){
                SgMesh::TriangleRef src = mesh->triangle(i);
                // Gets the vertex number of the i-th triangle in the Mesh object.
                Triangle tri;
                tri.indices[0] = vertexIndexTop + src[0];
                tri.indices[1] = vertexIndexTop + src[1];
                tri.indices[2] = vertexIndexTop + src[2];
                triangles.push_back(tri);
                // Adds to the triangular vertex number in the ODELink object.
            }
        }
    }

This completes the construction of the ODE model.

Next, we will explain the function that exchanges data with the ODE model. This is the source code of setKinematicStateToODE, which sets the position, orientation and velocity of the ODE Body object.

.. code-block:: cpp

    void ODELink::setKinematicStateToODE()
    {
        const Position& T = link->T();
        // Gets the positional and orientation matrix of the link.
    
        if(bodyID){
            // This is the process for movable objects.
        
            dMatrix3 R2 = { T(0,0), T(0,1), T(0,2), 0.0,
                            T(1,0), T(1,1), T(1,2), 0.0,
                            T(2,0), T(2,1), T(2,2), 0.0 };
    
            dBodySetRotation(bodyID, R2);
            //  Sets the orientation matrix.
        
            const Vector3 lc = link->R() * link->c();
            const Vector3 c = link->p() + lc;
            // Converts the link origin to the center of gravity.
        
            dBodySetPosition(bodyID, c.x(), c.y(), c.z());
            //Sets the position.
        
            const Vector3& w = link->w();
            const Vector3 v = link->v() + w.cross(lc);
            // Calculates the link's center of gravity and velocity.
        
            dBodySetLinearVel(bodyID, v.x(), v.y(), v.z());
            dBodySetAngularVel(bodyID, w.x(), w.y(), w.z());
            // Sets the velocity and angular velocity.

        }else{
            // This is the process for static objects. 
            // Updates the position of the shape data.
            for(vector<dGeomID>::iterator it = geomID.begin(); it!=geomID.end(); it++){
                OffsetMap::iterator it0 = offsetMap.find(*it);
                // For a primitive type, the positional and orientation matrix as viewed from the link's local coordinates is mapped,
               // so that matrix is applied.
                Position offset(Position::Identity());
                if(it0!=offsetMap.end())
                    offset = it0->second;
                Position T_ = T*offset;
                Vector3 p = T_.translation() + link->c();
                // Converts the link origin to the center of gravity.
                
                dMatrix3 R2 = { T(0,0), T(0,1), T(0,2), 0.0,
                                T(1,0), T(1,1), T(1,2), 0.0,
                                T(2,0), T(2,1), T(2,2), 0.0 };

                dGeomSetPosition(*it, p.x(), p.y(), p.z());
                dGeomSetRotation(*it, R2);
                // Updates the position and orientation information of the shape data.
            }
        }
    }

This is the source code of setTorqueToODE, which sets the torque of the ODE Body object.

.. code-block:: cpp

    void ODELink::setTorqueToODE()
    {
        if(link->isRotationalJoint()){
            // For when it is a rotational joint.
            dJointAddHingeTorque(jointID, link->u());
        } else if(link->isSlideJoint()){
            // For when it is a sliding joint.
            dJointAddSliderForce(jointID, link->u());
        }
    }


This is the source code of getKinematicStateFromODE, which gets the joint angle, angular velocity, link position and orientation, and velocity from the ODE Body object.

.. code-block:: cpp

    void ODELink::getKinematicStateFromODE()
    {
        if(jointID){
            // This is the process when there is a joint.
            if(link->isRotationalJoint()){
                // If it is a rotational joint, gets the angle and angular velocity.
                link->q() = dJointGetHingeAngle(jointID);
                link->dq() = dJointGetHingeAngleRate(jointID);
            } else if(link->isSlideJoint()){
                // If it is a sliding joint, gets the position and velocity.
                link->q() = dJointGetSliderPosition(jointID);
                link->dq() = dJointGetSliderPositionRate(jointID);
            }
        }

        const dReal* R = dBodyGetRotation(bodyID);
        // Gets the orientation matrix of the ODE Body.
    
        link->R() <<
            R[0], R[1], R[2],
            R[4], R[5], R[6],
            R[8], R[9], R[10];
        // Sets the orientation matrix of the Link object.
    
        typedef Eigen::Map<const Eigen::Matrix<dReal, 3, 1> > toVector3;
        const Vector3 c = link->R() * link->c();
        link->p() = toVector3(dBodyGetPosition(bodyID)) - c;
        // Gets the position of the ODE Body, converts from center of gravity to joint position,
        // and sets it to the Link object positional vector.
    
        link->w() = toVector3(dBodyGetAngularVel(bodyID));
        // Gets the angular velocity of the ODE Body and sets it to the angular velocity vector of the Link object.
    
        link->v() = toVector3(dBodyGetLinearVel(bodyID)) - link->w().cross(c);
        // Gets the velocity of the ODE Body, converts it to the velocity of the joint position,
        // and sets it to the velocity vector of the Link object.
    }

Collision detection
-------------------

In the ODESimulatorItem::stepSimulation function, you will find this line:  ::

    dSpaceCollide(spaceID, (void*)this, &nearCallback);

. This is an ODE function that searches for an object that may collide and calls the nearCallback function specified by the third argument. The second argument is used to pass parameters. In ODE, collision detection is performed in this way and binding force is created in the nearCallback function between objects that come into contact. Although we won’t get into a detailed explanation about ODE here, we will explain how to handle crawler links.

This is the source code for the nearCallback function.

.. code-block:: cpp

    static void nearCallback(void* data, dGeomID g1, dGeomID g2)
    {
        ...............

        ODESimulatorItemImpl* impl = (ODESimulatorItemImpl*)data;
        // Allows access to ODESimulatorItemImpl variables.

        ................
        if(numContacts > 0){
            // This is the process when there is contact.
            dBodyID body1ID = dGeomGetBody(g1);
            dBodyID body2ID = dGeomGetBody(g2);
            Link* crawlerlink = 0;
            if(!impl->crawlerLinks.empty()){
                CrawlerLinkMap::iterator p = impl->crawlerLinks.find(body1ID);
                if(p != impl->crawlerLinks.end()){
                    crawlerlink = p->second;
                }
                // Checks whether or not the touched link is a crawler type.
                // (At this point, it is assumed that there is no contact between crawler links.)
                ..............................
            }
            for(int i=0; i < numContacts; ++i){
                dSurfaceParameters& surface = contacts[i].surface;
                if(!crawlerlink){
                    surface.mode = dContactApprox1;
                    surface.mu = impl->friction;
                    // If it is not a crawler link, sets the frictional force.
                } else {
                    surface.mode = dContactFDir1 | dContactMotion1 | dContactMu2 | dContactApprox1_2;
                    // For crawler links, sets the surface velocity in friction direction 1 and the friction force in friction direction 2.
                    const Vector3 axis = crawlerlink->R() * crawlerlink->a();
                    // Calculates the rotational axis vector of the crawler link.
                    const Vector3 n(contacts[i].geom.normal);
                    // Gets the normal vector of the contact point.
                    Vector3 dir = axis.cross(n);
                    if(dir.norm() < 1.0e-5){
                        // When these two vectors are parallel, only the frictional force is set. 
                        surface.mode = dContactApprox1;
                        surface.mu = impl->friction;
                    } else {
                        dir *= sign;
                        dir.normalize();
                        contacts[i].fdir1[0] = dir[0];
                        contacts[i].fdir1[1] = dir[1];
                        contacts[i].fdir1[2] = dir[2];
                        // Sets the direction perpendicular to the two vectors to friction direction 1.
                        surface.motion1 = crawlerlink->u();
                        // Sets the surface velocity relative to friction direction 1.

                ............................

Sensor output
-------------

Next, we will explain about sensor output from force sensors, etc. The acceleration sensor, gyroscope, and force sensor attached to the robot are described in AccelSensor class, RateGyroSensor class, and ForceSensor class respectively. BasicSensorSimulationHelper is a utility class that summarizes the processes related to these sensors.

This is the source code of the processes related to sensors in the createBody function, which constructs the model.

.. code-block:: cpp
    
    sensorHelper.initialize(body, simImpl->timeStep, simImpl->gravity);
    // Performs initialization. The second argument is the increment time of the simulation, and the third argument is the gravity vector.
    
    // Then, we will configure the settings to obtain the force applied to the joints from ODE.
    const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
    forceSensorFeedbacks.resize(forceSensors.size());
    // Secures an area for storing data according to the number of force sensors.
    for(size_t i=0; i < forceSensors.size(); ++i){
        dJointSetFeedback(
            odeLinks[forceSensors[i]->link()->index()]->jointID, &forceSensorFeedbacks[i]);
        // The sensor object returns the link object to which the sensor is attached in the link() function.
        // Then we get the ODE joint id. Specify the data storage location relative to ODE.
    
Perform the following process in the stepSimulation function.

.. code-block:: cpp
    
    for(size_t i=0; i < activeSimBodies.size(); ++i){
        ODEBody* odeBody = static_cast<ODEBody*>(activeSimBodies[i]);

        if(!odeBody->sensorHelper.forceSensors().empty()){
            odeBody->updateForceSensors(flipYZ);
            // If there is a force sensor, calls the updateForceSensors class.
        }
        
        odeBody->getKinematicStateFromODE(flipYZ);
        
        if(odeBody->sensorHelper.hasGyroOrAccelSensors()){
            odeBody->sensorHelper.updateGyroAndAccelSensors();
            // If there is a gyroscope or acceleration sensor, calls updateGyroAndAccelSensors().
            // In this function, the output value of the sensor is calculated from the velocity and angular velocity of the Link object.
        }
    }

This is the source code for updateForceSensors.

.. code-block:: cpp
    
    void ODEBody::updateForceSensors(bool flipYZ)
    {
        const DeviceList<ForceSensor>& forceSensors = sensorHelper.forceSensors();
        // Gets the list of force sensors.
    
        for(int i=0; i < forceSensors.size(); ++i){
            ForceSensor* sensor = forceSensors.get(i);
            const Link* link = sensor->link();
            // You can get the pointer to the Link object to which the sensor is attached.
        
            const dJointFeedback& fb = forceSensorFeedbacks[i];
            Vector3 f, tau;
            f   << fb.f2[0], fb.f2[1], fb.f2[2];
            tau << fb.t2[0], fb.t2[1], fb.t2[2];
            // Gets the data on the force and torque applied to joints from ODE. 
 
            const Matrix3 R = link->R() * sensor->R_local();
            // With the R_local() function, you can get the orientation matrix of the sensor as seen from the coordinate system of the link where the sensor is attached.
            // Apply the orientation matrix of the link and convert it to the orientation matrix of the sensor as seen from the world coordinate system.
            const Vector3 p = link->R() * sensor->p_local();
            // The position of the sensor can be acquired in the same way, with the p_local() function.
            // Calculates the vector of the sensor position from the link origin as seen in the world coordinate system.

            sensor->f()   = R.transpose() * f;
            //  Converts to the sensor coordinate system and assigns it to the force data variable.
        
            sensor->tau() = R.transpose() * (tau - p.cross(f));
            // Using tau - p.cross(f), converts the torque around the link axis to the torque around the sensor position.
            // Converts it again ito the sensor coordinate system and assigns it to a torque data variable.

            sensor->notifyStateChange();
            // This is a function that sends a signal to notify that the sensor output has been updated.
    }   
        }
    }


About virtual joints
--------------------

By setting a virtual joint between two links, it is possible to generate a binding force between the specified links. With this you can simulate a closed link mechanism. share/model/misc/ClosedLinkSample.wrl is a sample of a closed link model.

In this sample model, the virtual joint definition is written as follows: ::

 DEF J1J3 ExtraJoint {
     link1Name "J1"
     link2Name "J3"
     link1LocalPos 0.2 0 0
     link2LocalPos 0 0.1 0
     jointType "piston"
     jointAxis 0 0 1
 }

J1J3 is the name given to the virtual joint. link1Name and link2Name specify the names of the two links to be bound together. The binding positions are specified in each link coordinate system with link1LocalPos and link2LocalPos. The type of binding is specified with jointType. You can set either **piston** or **ball**. In jointAxis, specify the binding axis as seen in the link coordinate system of link1.

These pieces of information are stored in the ExtraJoint structure of the Body object. The structure is defined as: ::

 struct ExtraJoint {
         ExtraJointType type;
         Vector3 axis;
         Link* link[2];
         Vector3 point[2];
 };

and the values defined in the model file are saved.

Next is the source code of setExtraJoint(), which sets up a virtual joint in the ODEBody object.

.. code-block:: cpp

    void ODEBody::setExtraJoints(bool flipYZ)
    {
        Body* body = this->body();
        const int n = body->numExtraJoints();
        // Gets the number of virtual joints.

        for(int j=0; j < n; ++j){
            Body::ExtraJoint& extraJoint = body->extraJoint(j);
            // Gets a reference to the virtual joint.

            ODELinkPtr odeLinkPair[2];
            for(int i=0; i < 2; ++i){
                ODELinkPtr odeLink;
                Link* link = extraJoint.link[i];
                // You can get pointers to the links to be bound by the virtual joint.
            
                if(link->index() < odeLinks.size()){
                    odeLink = odeLinks[link->index()];               
                    if(odeLink->link == link){
                        odeLinkPair[i] = odeLink;
                        // Saves the ODELink object corresponding to that Link object.
                    }
                }
                if(!odeLink){
                    break;
                }
            }

            if(odeLinkPair[1]){
                dJointID jointID = 0;
                Link* link = odeLinkPair[0]->link;
                Vector3 p = link->attitude() * extraJoint.point[0] + link->p();
                // Converts the binding position of Link1 to the world coordinate system.
            
                Vector3 a = link->attitude() * extraJoint.axis;
                // Converts the binding axis to the world coordinate system.
            
                if(extraJoint.type == Body::EJ_PISTON){
                    jointID = dJointCreatePiston(worldID, 0);
                    // Creates a piston joint.
                    dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                    // Connects two links to that joint.
                    dJointSetPistonAnchor(jointID, p.x(), p.y(), p.z());
                    // Sets the position of the joint.
                    dJointSetPistonAxis(jointID, a.x(), a.y(), a.z());
                    // Sets the joint axis.
                } else if(extraJoint.type == Body::EJ_BALL){
                    jointID = dJointCreateBall(worldID, 0);
                    // Creates a ball joint.
                    dJointAttach(jointID, odeLinkPair[0]->bodyID, odeLinkPair[1]->bodyID);
                    // Connects two links to the joint.
                    dJointSetBallAnchor(jointID, p.x(), p.y(), p.z());
                    // Sets the position of the joint.
                }
            }
        }
    }

Build
-----

Refer to the   :doc:`hello-world-sample` and the :doc:`sample1`  for information about the build method.

