/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreMaterial.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTexture.h>
#include <OgreRenderQueue.h>

#include "nav_msgs/msg/occupancy_grid.hpp"

namespace perception_msgs {
namespace displays {

/**
 * \class AttentionSwatch
 * \brief Enhanced swatch for displaying occupancy grid sections with advanced alpha handling
 */
class AttentionSwatch {
public:
  AttentionSwatch(
    Ogre::SceneManager * scene_manager,
    Ogre::SceneNode * parent_scene_node,
    size_t x, size_t y, size_t width, size_t height,
    float resolution, bool draw_under = false);

  ~AttentionSwatch();

  void updateAlpha(const Ogre::SceneBlendType & sceneBlending, bool depth_write, float alpha);
  void updateData(const nav_msgs::msg::OccupancyGrid & map);
  void setVisible(bool visible);
  void setRenderQueueGroup(uint8_t group);
  void setDepthWriteEnabled(bool depth_write_enabled);
  void setDrawOrder(bool draw_under);

  Ogre::Pass * getTechniquePass();
  std::string getTextureName();
  void resetOldTexture();

private:
  void resetTexture(Ogre::DataStreamPtr & pixel_stream);
  void setupMaterial();
  void setupSceneNodeWithManualObject();
  void setupSquareManualObject();
  void addPointWithPlaneCoordinates(float x, float y);

  Ogre::SceneManager * scene_manager_;
  Ogre::SceneNode * parent_scene_node_;
  Ogre::SceneNode * scene_node_;
  Ogre::ManualObject * manual_object_;

  size_t x_, y_, width_, height_;

  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
  Ogre::TexturePtr old_texture_;

  static size_t material_count_;
  static size_t map_count_;
  static size_t node_count_;
  static size_t texture_count_;
};

}  // namespace displays
}  // namespace perception_msgs
