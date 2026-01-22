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

#include "perception_msgs/displays/attention_grid/attention_swatch.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRenderable.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSharedPtr.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>

#include "rviz_rendering/custom_parameter_indices.hpp"

namespace perception_msgs {
namespace displays {

// Helper class to set alpha parameter on all renderables
class AttentionAlphaSetter : public Ogre::Renderable::Visitor
{
public:
  explicit AttentionAlphaSetter(float alpha)
  : alpha_vec_(alpha, alpha, alpha, alpha)
  {}

  void visit(Ogre::Renderable * rend, Ogre::ushort lodIndex, bool isDebug, Ogre::Any * pAny) override
  {
    (void) lodIndex;
    (void) isDebug;
    (void) pAny;
    rend->setCustomParameter(RVIZ_RENDERING_ALPHA_PARAMETER, alpha_vec_);
  }

private:
  Ogre::Vector4 alpha_vec_;
};

size_t AttentionSwatch::material_count_ = 0;
size_t AttentionSwatch::map_count_ = 0;
size_t AttentionSwatch::node_count_ = 0;
size_t AttentionSwatch::texture_count_ = 0;

AttentionSwatch::AttentionSwatch(
  Ogre::SceneManager * scene_manager,
  Ogre::SceneNode * parent_scene_node,
  size_t x, size_t y, size_t width, size_t height,
  float resolution, bool draw_under)
: scene_manager_(scene_manager),
  parent_scene_node_(parent_scene_node),
  manual_object_(nullptr),
  x_(x), y_(y), width_(width), height_(height)
{
  setupMaterial();
  setupSceneNodeWithManualObject();

  scene_node_->setPosition(x * resolution, y * resolution, 0.01f);
  scene_node_->setScale(width * resolution, height * resolution, 1.0);

  setDrawOrder(draw_under);

  // Don't show grid until the plugin is actually enabled
  manual_object_->setVisible(false);
}

AttentionSwatch::~AttentionSwatch()
{
  if (manual_object_) {
    scene_manager_->destroyManualObject(manual_object_);
  }
}

void AttentionSwatch::updateAlpha(
  const Ogre::SceneBlendType & sceneBlending, bool depth_write, float alpha)
{
  if (material_) {
    material_->setSceneBlending(sceneBlending);
    material_->setDepthWriteEnabled(depth_write);
  }
  
  if (manual_object_) {
    AttentionAlphaSetter alpha_setter(alpha);
    manual_object_->visitRenderables(&alpha_setter);
  }
}

void AttentionSwatch::updateData(const nav_msgs::msg::OccupancyGrid & map)
{
  size_t pixels_size = width_ * height_;
  size_t map_size = map.data.size();
  size_t map_width = map.info.width;

  auto pixels = std::vector<unsigned char>(pixels_size, 255);

  auto pixel_data = pixels.begin();
  for (size_t map_row = y_; map_row < y_ + height_; map_row++) {
    size_t pixel_index = map_row * map_width + x_;
    size_t pixels_to_copy = std::min(width_, map_size - pixel_index);

    auto row_start = map.data.begin() + pixel_index;
    std::copy(row_start, row_start + pixels_to_copy, pixel_data);
    pixel_data += pixels_to_copy;
    if (pixel_index + pixels_to_copy >= map_size) {
      break;
    }
  }

  Ogre::DataStreamPtr pixel_stream(new Ogre::MemoryDataStream(pixels.data(), pixels_size));

  resetTexture(pixel_stream);
  resetOldTexture();
}

void AttentionSwatch::setVisible(bool visible)
{
  if (manual_object_) {
    manual_object_->setVisible(visible);
  }
}

void AttentionSwatch::resetOldTexture()
{
  if (old_texture_) {
    Ogre::TextureManager::getSingleton().remove(old_texture_);
    old_texture_.reset();
  }
}

void AttentionSwatch::setRenderQueueGroup(uint8_t group)
{
  if (manual_object_) {
    manual_object_->setRenderQueueGroup(group);
  }
}

void AttentionSwatch::setDepthWriteEnabled(bool depth_write_enabled)
{
  if (material_) {
    material_->setDepthWriteEnabled(depth_write_enabled);
  }
}

void AttentionSwatch::setDrawOrder(bool draw_under)
{
  if (manual_object_) {
    manual_object_->setRenderQueueGroup(draw_under ? Ogre::RENDER_QUEUE_4 : Ogre::RENDER_QUEUE_MAIN);
  }

  if (material_) {
    // Positive bias pushes the swatch slightly away from the camera so foreground geometry remains visible
    const float bias = draw_under ? 1.0f : -1.0f;
    material_->setDepthBias(bias, 0.0f);
  }
}

Ogre::Pass * AttentionSwatch::getTechniquePass()
{
  if (material_) {
    return material_->getTechnique(0)->getPass(0);
  }
  return nullptr;
}

std::string AttentionSwatch::getTextureName()
{
  if (texture_) {
    return texture_->getName();
  }
  return "";
}

void AttentionSwatch::resetTexture(Ogre::DataStreamPtr & pixel_stream)
{
  old_texture_ = texture_;

  texture_ = Ogre::TextureManager::getSingleton().loadRawData(
    "AttentionGridTexture" + std::to_string(texture_count_++),
    "rviz_rendering",
    pixel_stream,
    static_cast<uint16_t>(width_), static_cast<uint16_t>(height_),
    Ogre::PF_L8, Ogre::TEX_TYPE_2D, 0);
}

void AttentionSwatch::setupMaterial()
{
  material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
  material_ = material_->clone("AttentionGridMaterial" + std::to_string(material_count_++));

  material_->setReceiveShadows(false);
  material_->getTechnique(0)->setLightingEnabled(false);
  material_->setDepthBias(0.0f, 0.0f);
  material_->setCullingMode(Ogre::CULL_NONE);
  material_->setDepthWriteEnabled(false);
}

void AttentionSwatch::setupSceneNodeWithManualObject()
{
  manual_object_ = scene_manager_->createManualObject("AttentionGridObject" + std::to_string(map_count_++));

  scene_node_ = parent_scene_node_->createChildSceneNode(
    "AttentionGridNode" + std::to_string(node_count_++));
  scene_node_->attachObject(manual_object_);

  setupSquareManualObject();
}

void AttentionSwatch::setupSquareManualObject()
{
  manual_object_->begin(
    material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST, "rviz_rendering");

  // First triangle
  addPointWithPlaneCoordinates(0.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 1.0f);
  addPointWithPlaneCoordinates(0.0f, 1.0f);

  // Second triangle
  addPointWithPlaneCoordinates(0.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 0.0f);
  addPointWithPlaneCoordinates(1.0f, 1.0f);

  manual_object_->end();
}

void AttentionSwatch::addPointWithPlaneCoordinates(float x, float y)
{
  manual_object_->position(x, y, 0.0f);
  manual_object_->textureCoord(x, y);
  manual_object_->normal(0.0f, 0.0f, 1.0f);
}

}  // namespace displays
}  // namespace perception_msgs
