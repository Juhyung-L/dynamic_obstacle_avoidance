#include <OgreManualObject.h>

#include "rviz_common/validate_floats.hpp"
#include "rviz_common/msg_conversions.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_rendering/material_manager.hpp"

#include "gnc_rviz_plugins/path2d_list_display.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"

namespace gnc_rviz_plugins
{
Path2DListDisplay::Path2DListDisplay(rviz_common::DisplayContext* display_context)
: Path2DListDisplay()
{
    context_ = display_context;
    scene_manager_ = context_->getSceneManager();
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
}

Path2DListDisplay::Path2DListDisplay()
{
    alpha_property_ = new rviz_common::properties::FloatProperty(
        "Alpha", 1.0,
        "Amount of transparency to apply to the path.", this);
    color_.a = alpha_property_->getFloat();

    offset_property_ = new rviz_common::properties::VectorProperty(
        "Offset", Ogre::Vector3::ZERO,
        "Allows you to offset the path from the origin of the reference frame.  In meters.",
        this, SLOT(updateOffset()));

    static int count = 0;
    std::string material_name = "Path2DMaterial" + std::to_string(count++);
    lines_material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(material_name);
}

Path2DListDisplay::~Path2DListDisplay()
{
    destroyObjects();
}

void Path2DListDisplay::onInitialize()
{
    MFDClass::onInitialize();

    buffer_length_ = 1;
    updateBufferLength();
}

void Path2DListDisplay::reset()
{
    MFDClass::reset();
    updateBufferLength();
}

// this function is called everytime a new message arrives and needs to be displayed
void Path2DListDisplay::processMessage(nav_2d_msgs::msg::Path2DList::ConstSharedPtr msg)
{
    // check if message has valid floats
    if (!validateMsg(*msg))
    {
        setStatus(rviz_common::properties::StatusProperty::Error,
            "Topic",
            "Message contained invalid floating point values (nans or infs)");
        return;
    }
    
    /**
     * RViz has a fixed frame that you can set.
     * This code block checks if there is a transform between the fixed frame and the frame of the
     * message you are trying to display.
     * If there is no transform available, function exits permaturely
    */
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(msg->header, position, orientation))
    {
        setMissingTransformToFixedFrame(msg->header.frame_id);
        return ;
    }
    setTransformOk();

    Ogre::Matrix4 transform(orientation);
    transform.setTrans(position);

    // need detach and reattach all manual objects
    // to prevent previous path drawings from lingering on the scene
    scene_node_->detachAllObjects();
    for (auto& manual_object : manual_objects_)
    {
        manual_object->clear();
        scene_node_->attachObject(manual_object);
    }

    // make sure msg->paths.size() == manual_objects_.size()
    // this will be called once since the number of paths in the message will remain constant
    if (buffer_length_ != msg->paths.size())
    {
        buffer_length_ = msg->paths.size();
        updateBufferLength();
    }

    // update manual objects
    Ogre::ManualObject* manual_object = nullptr;
    nav_2d_msgs::msg::Path2D path;
    Ogre::Vector3 ogre_pose;
    ogre_pose.z = 0.0;
    for (size_t i=0 ; i<buffer_length_; ++i)
    {
        manual_object = manual_objects_[i];
        path = msg->paths[i];
        manual_object->estimateVertexCount(path.poses.size());
        manual_object->begin(
            lines_material_->getName(), Ogre::RenderOperation::OT_LINE_STRIP, "rviz_rendering");
        for (auto& pose : path.poses)
        {
            ogre_pose.x = pose.x;
            ogre_pose.y = pose.y;
            manual_object->position(transform * ogre_pose);
            rviz_rendering::MaterialManager::enableAlphaBlending(lines_material_, color_.a);
            manual_object->colour(color_);
        }
        manual_object->end();
    }

    // trigger re-render
    context_->queueRender();
}

bool Path2DListDisplay::validateMsg(const nav_2d_msgs::msg::Path2DList& msg)
{
    for (auto& path : msg.paths)
    {
        for (auto& pose : path.poses)
        {
            if (!rviz_common::validateFloats(pose.x) ||
                !rviz_common::validateFloats(pose.y) ||
                !rviz_common::validateFloats(pose.theta))
            {
                return false;
            }
        }
    }
    return true;
}

void Path2DListDisplay::updateOffset()
{
    scene_node_->setPosition(offset_property_->getVector());
    context_->queueRender();
}

void Path2DListDisplay::destroyObjects()
{
    for (auto manual_object : manual_objects_)
    {
        manual_object->clear();
        scene_manager_->destroyManualObject(manual_object);
    }
    manual_objects_.clear();
}

void Path2DListDisplay::updateBufferLength()
{
    destroyObjects();

    manual_objects_.reserve(buffer_length_);
    for (size_t i=0; i<buffer_length_; ++i)
    {
        auto manual_object = scene_manager_->createManualObject();
        manual_objects_.push_back(manual_object);
    }
}
}

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(gnc_rviz_plugins::Path2DListDisplay, rviz_common::Display)