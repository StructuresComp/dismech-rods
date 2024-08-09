#include "magnumDERSimulationEnvironment.h"
#include "eigenIncludes.h"

namespace Magnum {


magnumRenderer::magnumRenderer(const shared_ptr<world>& m_world, const simParams& sim_params,
                               const shared_ptr<worldLogger>& logger, int argc, char **argv)
                               : Platform::Application({argc, argv}, NoCreate), derSimulationEnvironment(m_world, sim_params, logger) {
    const Vector2 dpiScaling = this->dpiScaling({});
    Configuration conf;
    conf.setTitle("Magnum Mouse Interaction Example")
        .setSize(conf.size(), dpiScaling);
    GLConfiguration glConf;
    glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
    if (!tryCreate(conf, glConf))
        create(conf, glConf.setSampleCount(0));

    _vertexColorShader = Shaders::VertexColorGL3D{};
    _flatShader = Shaders::FlatGL3D{};
    _shader = Shaders::PhongGL{};
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);

    const struct {
        Vector3 pos;
        Color3 color;
    } data[]{{{-1.0f, -1.0f, 0.0f}, 0xff0000_rgbf},
             {{1.0f,  -1.0f, 0.0f}, 0x00ff00_rgbf},
             {{0.0f,  1.0f,  0.0f}, 0x0000ff_rgbf}};

    GL::Buffer buffer;
    buffer.setData(data);
    _mesh = GL::Mesh{};
    _mesh.setCount(3)
         .addVertexBuffer(std::move(buffer), 0,
                          Shaders::VertexColorGL3D::Position{},
                          Shaders::VertexColorGL3D::Color3{});

    auto triangle = new Object3D{&_scene};
    new VertexColorDrawable{*triangle, _vertexColorShader, _mesh, _drawables};

    _grid = MeshTools::compile(Primitives::grid3DWireframe({15, 15}));
    auto grid = new Object3D{&_scene};
    (*grid).rotateX(90.0_degf).scale(Vector3{8.0f});
    new FlatDrawable{*grid, _flatShader, _grid, _drawables};

    auto cyl = Primitives::capsule3DSolid(5, 5, 5, 1.0);
    cyl = MeshTools::transform3D(cyl, Matrix4::scaling({0.3f, 0.3f, 0.3f}));
    _cylinder = MeshTools::compile(cyl);
    cylinder = new Object3D{&_scene};
//    new CylinderDrawable{*cylinder, _shader, _cylinder, _drawables};

    // TODO(asjchoi) set the number of edges here dynamically
    for (int i = 0; i < 200; i++) {
        edges.emplace_back(std::make_unique<Object3D>(&_scene));
        cylinderDrawables.emplace_back(std::make_unique<CylinderDrawable>(*cylinder, _shader, _cylinder, _drawables));
    }

    _cameraObject = new Object3D{&_scene};
    (*_cameraObject)
        .translate(Vector3::zAxis(5.0f))
        .rotateX(-15.0_degf)
        .rotateY(30.0_degf);
    _camera = new SceneGraph::Camera3D{*_cameraObject};
    _camera->setProjectionMatrix(Matrix4::perspectiveProjection(
        45.0_degf, Vector2{windowSize()}.aspectRatio(), 0.01f, 100.0f));

    _lastDepth = ((_camera->projectionMatrix() * _camera->cameraMatrix()).transformPoint({}).z() + 1.0f) * 0.5f;
}

Float magnumRenderer::depthAt(const Vector2i &windowPosition) {
    const Vector2i position = windowPosition * Vector2{framebufferSize()} / Vector2{windowSize()};
    const Vector2i fbPosition{position.x(), GL::defaultFramebuffer.viewport().sizeY() - position.y() - 1};

    GL::defaultFramebuffer.mapForRead(GL::DefaultFramebuffer::ReadAttachment::Front);
    Image2D data = GL::defaultFramebuffer.read(
        Range2Di::fromSize(fbPosition, Vector2i{1}).padded(Vector2i{2}),
        {GL::PixelFormat::DepthComponent, GL::PixelType::Float});

    return Math::min<Float>(data.pixels<Float>().asContiguous());
}

Vector3 magnumRenderer::unproject(const Vector2i &windowPosition, Float depth) const {
    const Vector2i viewSize = windowSize();
    const Vector2i viewPosition{windowPosition.x(), viewSize.y() - windowPosition.y() - 1};
    const Vector3 in{2 * Vector2{viewPosition} / Vector2{viewSize} - Vector2{1.0f}, depth * 2.0f - 1.0f};

    return _camera->projectionMatrix().inverted().transformPoint(in);
}

void magnumRenderer::keyPressEvent(KeyEvent &event) {
    if (event.key() == KeyEvent::Key::Zero || event.key() == KeyEvent::Key::NumZero) {
        (*_cameraObject)
            .resetTransformation()
            .translate(Vector3::zAxis(5.0f))
            .rotateX(-15.0_degf)
            .rotateY(30.0_degf);
        redraw();
        return;
    } else if (event.key() == KeyEvent::Key::One || event.key() == KeyEvent::Key::NumOne ||
               event.key() == KeyEvent::Key::Three || event.key() == KeyEvent::Key::NumThree ||
               event.key() == KeyEvent::Key::Seven || event.key() == KeyEvent::Key::NumSeven) {
        const Vector3 viewTranslation = _cameraObject->transformation().rotationScaling().inverted() *
                                        _cameraObject->transformation().translation();

        const Float multiplier = event.modifiers() & KeyEvent::Modifier::Ctrl ? -1.0f : 1.0f;

        Matrix4 transformation;
        if (event.key() == KeyEvent::Key::Seven || event.key() == KeyEvent::Key::NumSeven)
            transformation = Matrix4::rotationX(-90.0_degf * multiplier);
        else if (event.key() == KeyEvent::Key::One || event.key() == KeyEvent::Key::NumOne)
            transformation = Matrix4::rotationY(90.0_degf - 90.0_degf * multiplier);
        else if (event.key() == KeyEvent::Key::Three || event.key() == KeyEvent::Key::NumThree)
            transformation = Matrix4::rotationY(90.0_degf * multiplier);
        else
            CORRADE_INTERNAL_ASSERT_UNREACHABLE();

        _cameraObject->setTransformation(transformation * Matrix4::translation(viewTranslation));
        redraw();
    }
}

void magnumRenderer::mousePressEvent(MouseEvent &event) {
    if (event.button() != MouseEvent::Button::Left && event.button() != MouseEvent::Button::Middle)
        return;

    const Float currentDepth = depthAt(event.position());
    const Float depth = currentDepth == 1.0f ? _lastDepth : currentDepth;
    _translationPoint = unproject(event.position(), depth);
    if (currentDepth != 1.0f || _rotationPoint.isZero()) {
        _rotationPoint = _translationPoint;
        _lastDepth = depth;
    }

    redraw();
}

void magnumRenderer::mouseMoveEvent(MouseMoveEvent &event) {
    if (_lastPosition == Vector2i{-1}) _lastPosition = event.position();
    const Vector2i delta = event.position() - _lastPosition;
    _lastPosition = event.position();

    if (!event.buttons()) return;

    if (event.modifiers() & MouseMoveEvent::Modifier::Shift) {
        const Vector3 p = unproject(event.position(), _lastDepth);
        _cameraObject->translateLocal(_translationPoint - p);
        _translationPoint = p;
    } else {
        _cameraObject->transformLocal(
            Matrix4::translation(_rotationPoint) *
            Matrix4::rotationX(-0.01_radf * delta.y()) *
            Matrix4::rotationY(-0.01_radf * delta.x()) *
            Matrix4::translation(-_rotationPoint));
    }

    redraw();
}

void magnumRenderer::mouseScrollEvent(MouseScrollEvent &event) {
    const Float currentDepth = depthAt(event.position());
    const Float depth = currentDepth == 1.0f ? _lastDepth : currentDepth;
    const Vector3 p = unproject(event.position(), depth);
    if (currentDepth != 1.0f || _rotationPoint.isZero()) {
        _rotationPoint = p;
        _lastDepth = depth;
    }

    const Float direction = event.offset().y();
    if (!direction) return;

    _cameraObject->translateLocal(_rotationPoint * direction * 0.1f);

    event.setAccepted();
    redraw();
}

void magnumRenderer::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

    _camera->draw(_drawables);

    swapBuffers();

    redraw();
}

//void magnumRenderer::tickEvent() {
//    static int i = 0;
//    if (i % 2 == 0) {
//        (*cylinder).translate(Vector3{1.0f, 0.0f, 0.0f});
//    } else {
//        (*cylinder).translate(Vector3{-1.0f, 0.0f, 0.0f});
//    }
//    i++;
//
//    redraw();
//}

void magnumRenderer::runSimulation() {
    while (w_p->simulationRunning()) {
        try {
            w_p->updateTimeStep();
        }
        catch (std::runtime_error &excep) {
            if (verbosity >= 1) {
                std::cout << "Caught a runtime_error when trying to world->updateTimeStep: " << excep.what() << std::endl;
                std::cout << "Attempting clean shutdown..." << std::endl;
            }
            cleanShutdown(logger_p, is_logging);
        }

//        Eigen::Vector3d top = w_p->soft_robots->limbs[0]->getVertex(190);
//        Eigen::Vector3d bot = w_p->soft_robots->limbs[0]->getVertex(189);
//        Eigen::Vector3d center_line_eigen = top - bot;
//        Eigen::Vector3d center_pos_eigen = (top + bot) * 0.5;
//
//        Magnum::Vector3 center_line(center_line_eigen.x(), center_line_eigen.y(), center_line_eigen.z());
//        Magnum::Vector3 center_pos(center_pos_eigen.x(), center_pos_eigen.y(), center_pos_eigen.z());
//
//        float height = center_line.length();
//        Magnum::Vector3 axis = center_line.normalized() ;
//
//        Matrix4 translation = Matrix4::translation(center_pos);
//        Matrix4 rotation = Matrix4::rotation(Math::angle(Vector3::zAxis(), axis),
//                                             Math::cross(Vector3::zAxis(), axis).normalized());
//
//        auto tf = translation * rotation * Matrix4::scaling(Vector3{0.1f, 0.1f, height});
//
//        cylinder->setTransformation(tf);
//
//        std::cout << center_line_eigen << std::endl;

        int limb_idx = 0;
        for (const auto &limb: w_p->soft_robots->limbs) {
            for (int i = 0; i < limb->ne; i++) {
                if (limb->isEdgeJoint[i] == 0) {
                    Eigen::Vector3d top = limb->getVertex(i+1);
                    Eigen::Vector3d bot = limb->getVertex(i);
                    Eigen::Vector3d center_line_eigen = top - bot;
                    Eigen::Vector3d center_pos_eigen = (top + bot) * 0.5;

                    Magnum::Vector3 center_line(center_line_eigen.x(), center_line_eigen.y(), center_line_eigen.z());
                    Magnum::Vector3 center_pos(center_pos_eigen.x(), center_pos_eigen.y(), center_pos_eigen.z());

                    float height = center_line.length();
                    Magnum::Vector3 axis = center_line.normalized() ;

                    Matrix4 translation = Matrix4::translation(center_pos);
                    Matrix4 rotation = Matrix4::rotation(Math::angle(Vector3::zAxis(), axis),
                                                         Math::cross(Vector3::zAxis(), axis).normalized());

                    Matrix4 tf = translation * rotation * Matrix4::scaling(Vector3{0.1f, 0.1f, height});

                    edges[i]->setTransformation(tf);

                    std::cout << center_line_eigen << std::endl;

                }
            }
            limb_idx++;
        }
        mainLoopIteration();
    }
}

VertexColorDrawable::VertexColorDrawable(Object3D &object, Shaders::VertexColorGL3D &shader, GL::Mesh &mesh,
                                         SceneGraph::DrawableGroup3D &drawables) : SceneGraph::Drawable3D{object, &drawables},
                                                                                   _shader(shader), _mesh(mesh) {}

void VertexColorDrawable::draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) {
    _shader
        .setTransformationProjectionMatrix(camera.projectionMatrix() * transformation)
        .draw(_mesh);
}

FlatDrawable::FlatDrawable(Object3D &object, Shaders::FlatGL3D &shader, GL::Mesh &mesh,
                           SceneGraph::DrawableGroup3D &drawables) : SceneGraph::Drawable3D{object, &drawables},
                                                                     _shader(shader), _mesh(mesh) {}

void FlatDrawable::draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) {
    _shader
        .setColor(0x747474_rgbf)
        .setTransformationProjectionMatrix(camera.projectionMatrix() * transformation)
        .draw(_mesh);
}

CylinderDrawable::CylinderDrawable(Object3D &object, Shaders::PhongGL &shader, GL::Mesh &mesh,
                                   SceneGraph::DrawableGroup3D &drawables) : SceneGraph::Drawable3D{object, &drawables},
                                                                             _shader(shader), _mesh(mesh),
                                                                             _color(Color3::fromHsv({35.0_degf, 1.0f, 1.0f})) {}

void CylinderDrawable::draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) {
    _shader.setLightPositions({{1.4f, 1.0f, 0.75f, 0.0f}})
        .setDiffuseColor(_color)
        .setAmbientColor(Color3::fromHsv({_color.hue(), 1.0f, 0.3f}))
        .setTransformationMatrix(transformation)
        .setNormalMatrix(transformation.normalMatrix())
        .setProjectionMatrix(camera.projectionMatrix())
        .draw(_mesh);
}

} // namespace Magnum
