#include "magnumDERSimulationEnvironment.h"
#include "eigenIncludes.h"


/*
 * The majority of this code's mouse interaction logic has been adapted from
 * https://doc.magnum.graphics/magnum/mouseinteraction_2MouseInteractionExample_8cpp-example.html
 */


namespace Magnum {


magnumDERSimulationEnvironment::magnumDERSimulationEnvironment(const shared_ptr<world>& m_world,
                                                               const renderParams& render_params,
                                                               const shared_ptr<worldLogger>& logger,
                                                               int argc, char **argv)
                               : Platform::Application({argc, argv}, NoCreate),
                                 derSimulationEnvironment(m_world, render_params, logger) {
    const Vector2 dpi_scaling = this->dpiScaling({});
    Configuration conf;
    conf.setTitle("DisMech")
//        .setSize(conf.size(), dpiScaling);
        .setSize({1280, 720}, dpi_scaling);
    GLConfiguration gl_conf;
    gl_conf.setSampleCount(dpi_scaling.max() < 2.0f ? 8 : 2);
    if (!tryCreate(conf, gl_conf))
        create(conf, gl_conf.setSampleCount(0));

    // Setup background color to off-white.
    GL::Renderer::setClearColor(0xFAF9F6_rgbf);

    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    render_scale = (float)render_params.render_scale;
    render_per = render_params.render_per;
    render_record_path = render_params.render_record_path;

    // Render a floor if necessary. If not, turn on face culling for improved speed.
    if (w_p->floorExists()) {
        auto floor_z = (float)(w_p->getFloorZ() * render_scale);
        floor_mesh = MeshTools::compile(Primitives::planeSolid());
        floor = std::make_unique<Object3D>(&scene);
        floor->
            scale(Vector3{8.0f})
            .rotateX(90.0_degf)
            .translate(Vector3(0.0f, floor_z, 0.0f));

        misc_drawables.push_back(std::make_unique<FlatDrawable>(*floor,
                                                                flat_shader,
                                                                floor_mesh,
                                                                drawables,
                                                                0xE0E0E0_rgbf));

        grid_mesh = MeshTools::compile(Primitives::grid3DWireframe({15, 15}));
        grid= std::make_unique<Object3D>(&scene);
        grid->
            scale(Vector3{8.0f})
            .rotateX(90.0_degf)
            .translate(Vector3(0.0f, floor_z, 0.0f));

        misc_drawables.push_back(std::make_unique<FlatDrawable>(*grid,
                                                                flat_shader,
                                                                grid_mesh,
                                                                drawables,
                                                                0x000000_rgbf));
    }
    else {
        GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    }

    vertex_color_shader = Shaders::VertexColorGL3D{};
    flat_shader = Shaders::FlatGL3D{};
    phong_shader = Shaders::PhongGL{};

    coordinate_frame = std::make_unique<Object3D>(&scene);
    coordinate_frame->scale(Vector3{2.0f});
    misc_drawables.push_back(std::make_unique<CoordinateFrame>(*coordinate_frame,
                                                               vertex_color_shader,
                                                               drawables));

    camera_object = std::make_unique<Object3D>(&scene);
    camera_object->
        translate(Vector3::zAxis(5.0f))
        .rotateX(-15.0_degf)
        .rotateY(30.0_degf);
    camera = std::make_unique<SceneGraph::Camera3D>(*camera_object);
    camera->setProjectionMatrix(Matrix4::perspectiveProjection(45.0_degf, 
                                                               Vector2{windowSize()}.aspectRatio(), 
                                                               0.01f, 100.0f));

    last_depth = ((camera->projectionMatrix() * camera->cameraMatrix()).transformPoint({}).z() + 1.0f) * 0.5f;
}

Float magnumDERSimulationEnvironment::depthAt(const Vector2i &windowPosition) {
    const Vector2i position = windowPosition * Vector2{framebufferSize()} / Vector2{windowSize()};
    const Vector2i fbPosition{position.x(), GL::defaultFramebuffer.viewport().sizeY() - position.y() - 1};

    GL::defaultFramebuffer.mapForRead(GL::DefaultFramebuffer::ReadAttachment::Front);
    Image2D data = GL::defaultFramebuffer.read(
        Range2Di::fromSize(fbPosition, Vector2i{1}).padded(Vector2i{2}),
        {GL::PixelFormat::DepthComponent, GL::PixelType::Float});

    return Math::min<Float>(data.pixels<Float>().asContiguous());
}

Vector3 magnumDERSimulationEnvironment::unproject(const Vector2i &windowPosition, Float depth) const {
    const Vector2i viewSize = windowSize();
    const Vector2i viewPosition{windowPosition.x(), viewSize.y() - windowPosition.y() - 1};
    const Vector3 in{2 * Vector2{viewPosition} / Vector2{viewSize} - Vector2{1.0f}, depth * 2.0f - 1.0f};

    return camera->projectionMatrix().inverted().transformPoint(in);
}

void magnumDERSimulationEnvironment::keyPressEvent(KeyEvent &event) {
    if (event.key() == KeyEvent::Key::Zero || event.key() == KeyEvent::Key::NumZero) {
        camera_object->
            resetTransformation()
            .translate(Vector3::zAxis(5.0f))
            .rotateX(-15.0_degf)
            .rotateY(30.0_degf);
        redraw();
        return;
    } else if (event.key() == KeyEvent::Key::One || event.key() == KeyEvent::Key::NumOne ||
               event.key() == KeyEvent::Key::Three || event.key() == KeyEvent::Key::NumThree ||
               event.key() == KeyEvent::Key::Seven || event.key() == KeyEvent::Key::NumSeven) {
        const Vector3 viewTranslation = camera_object->transformation().rotationScaling().inverted() *
                                        camera_object->transformation().translation();

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

        camera_object->setTransformation(transformation * Matrix4::translation(viewTranslation));
        redraw();
    }
}

void magnumDERSimulationEnvironment::mousePressEvent(MouseEvent &event) {
    if (event.button() != MouseEvent::Button::Left && event.button() != MouseEvent::Button::Middle)
        return;

    const Float current_depth = depthAt(event.position());
    const Float depth = current_depth == 1.0f ? last_depth : current_depth;
    translation_point = unproject(event.position(), depth);
    if (current_depth != 1.0f || rotation_point.isZero()) {
        rotation_point = translation_point;
        last_depth = depth;
    }

    redraw();
}

void magnumDERSimulationEnvironment::mouseMoveEvent(MouseMoveEvent &event) {
    if (last_position == Vector2i{-1}) last_position = event.position();
    const Vector2i delta = event.position() - last_position;
    last_position = event.position();

    if (!event.buttons()) return;

    if (event.modifiers() & MouseMoveEvent::Modifier::Shift) {
        const Vector3 p = unproject(event.position(), last_depth);
        camera_object->translateLocal(translation_point - p);
        translation_point = p;
    } else {
        camera_object->transformLocal(
            Matrix4::translation(rotation_point) *
            Matrix4::rotationX(-0.01_radf * delta.y()) *
            Matrix4::rotationY(-0.01_radf * delta.x()) *
            Matrix4::translation(-rotation_point));
    }

    redraw();
}

void magnumDERSimulationEnvironment::mouseScrollEvent(MouseScrollEvent &event) {
    const Float current_depth = depthAt(event.position());
    const Float depth = current_depth == 1.0f ? last_depth : current_depth;
    const Vector3 p = unproject(event.position(), depth);
    if (current_depth != 1.0f || rotation_point.isZero()) {
        rotation_point = p;
        last_depth = depth;
    }

    const Float direction = event.offset().y();
    if (!direction) return;

    camera_object->translateLocal(rotation_point * direction * 0.1f);

    event.setAccepted();
    redraw();
}

void magnumDERSimulationEnvironment::drawEvent() {
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color | GL::FramebufferClear::Depth);

    camera->draw(drawables);

    swapBuffers();

    redraw();
}


void magnumDERSimulationEnvironment::renderEdge(Eigen::Vector3d top_vertex,
                                                Eigen::Vector3d bot_vertex,
                                                float radius, Magnum::Color3 color) {
    top_vertex *= render_scale;
    bot_vertex *= render_scale;
    radius *= render_scale;

    Eigen::Vector3d center_line_eigen = top_vertex - bot_vertex;
    Eigen::Vector3d center_pos_eigen = 0.5 * (top_vertex + bot_vertex);

    // Magnum uses a right-handed coordinate system where y points up
    Magnum::Vector3 center_line(center_line_eigen.x(), center_line_eigen.z(), -center_line_eigen.y());
    Magnum::Vector3 center_pos(center_pos_eigen.x(), center_pos_eigen.z(), -center_pos_eigen.y());

    float length = center_line.length();
    Magnum::Vector3 axis = center_line.normalized() ;

    // https://doc.magnum.graphics/magnum/namespaceMagnum_1_1Primitives.html#add456eac5c394549b21ae2ae88f697b0
    float half_length = 0.5f * length / radius;

    capsule_mesh = MeshTools::compile(Primitives::capsule3DSolid(5, 5, 16, half_length));
    edges.push_back(std::make_unique<Object3D>(&scene));
    capsule_drawables.push_back(std::make_unique<CapsuleDrawable>(*edges.back(),
                                                                  phong_shader,
                                                                  capsule_mesh,
                                                                  drawables,
                                                                  color));

    Magnum::Matrix4 translation = Matrix4::translation(center_pos);
    Magnum::Matrix4 rotation = Matrix4::rotation(Math::angle(Vector3::yAxis(), axis),
                                                 Math::cross(Vector3::yAxis(), axis).normalized());

    Magnum::Matrix4 tf = translation * rotation * Matrix4::scaling(Vector3{radius});

    edges.back()->setTransformation(tf);
}


void magnumDERSimulationEnvironment::runSimulation() {
    int curr_iter = -1;

    PluginManager::Manager<Trade::AbstractImageConverter> manager;
    Containers::Pointer<Trade::AbstractImageConverter> converter;
    bool record_frames = !render_record_path.empty();
    if (record_frames) {
        // Initialize the PNG image converter
        converter = manager.loadAndInstantiate("PngImageConverter");
        if (!converter) {
            Error{} << "Cannot load the PngImageConverter plugin.";
        }
    }

    while (w_p->simulationRunning()) {
        curr_iter++;
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

        // Output info to cmdline
        cmdlineOutputHelper();

        if (curr_iter % render_per != 0)
            continue;

        capsule_drawables.clear();
        edges.clear();

        for (const auto &limb: w_p->soft_robots->limbs) {
            float radius = limb->rod_radius;
            for (int i = 0; i < limb->ne; i++) {
                if (limb->isEdgeJoint[i] != 0) {
                    continue;
                }
                renderEdge(limb->getVertex(i+1),
                           limb->getVertex(i),
                           radius,
                           Color3::fromHsv({210.0_degf, 0.75f, 1.0f}));
            }
        }

        int n, l;
        for (const auto &joint : w_p->soft_robots->joints) {
            for (int i = 0; i < joint->ne; i++) {
                n = joint->connected_nodes[i].first;
                l = joint->connected_nodes[i].second;

                renderEdge(w_p->soft_robots->limbs[l]->getVertex(n),
                           joint->x,
                           w_p->soft_robots->limbs[l]->rod_radius,
                           Color3::fromHsv({240.0_degf, 0.75f, 1.0f}));
            }
        }
        // Perform Magnum simulation loop
        mainLoopIteration();

        if (record_frames) {
            // Capture the frame
            Image2D image = GL::defaultFramebuffer.read(GL::defaultFramebuffer.viewport(), {PixelFormat::RGBA8Unorm});

            std::ostringstream filename;
            filename << render_record_path << "frame_" << std::setw(5) << std::setfill('0') << curr_iter << ".png";

            // Convert and save the image as a PNG file
            if (!converter->convertToFile(image, filename.str())) {
                Error{} << "Failed to save the image";
            }
        }
    }
}


FlatDrawable::FlatDrawable(Object3D &object, Shaders::FlatGL3D &shader, GL::Mesh &mesh,
                           SceneGraph::DrawableGroup3D &drawables, Color3 color) :
                           SceneGraph::Drawable3D{object, &drawables},
                           shader(shader), mesh(mesh), color(color) {}

void FlatDrawable::draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) {
    shader
        .setColor(color)
        .setTransformationProjectionMatrix(camera.projectionMatrix() * transformation)
        .draw(mesh);
}

CapsuleDrawable::CapsuleDrawable(Magnum::Object3D &object, Shaders::PhongGL &shader, GL::Mesh &mesh,
                                 SceneGraph::DrawableGroup3D &drawables, Color3 color) :
                                 SceneGraph::Drawable3D{object, &drawables},
                                 shader(shader), mesh(mesh),
                                 color(color) {}

void CapsuleDrawable::draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) {
    shader.setLightPositions({{1.4f, 1.0f, 0.75f, 0.0f}})
        .setDiffuseColor(color)
        .setAmbientColor(Color3::fromHsv({color.hue(), 0.75f, 0.3f}))
        .setTransformationMatrix(transformation)
        .setNormalMatrix(transformation.normalMatrix())
        .setProjectionMatrix(camera.projectionMatrix())
        .draw(mesh);
}

CoordinateFrame::CoordinateFrame(Object3D &object, Shaders::VertexColorGL3D &shader,
                                 SceneGraph::DrawableGroup3D &drawables) :
                                 SceneGraph::Drawable3D{object, &drawables},
                                 shader(shader) {
    mesh = MeshTools::compile(Primitives::axis3D());
}

void CoordinateFrame::draw(const Matrix4 &transformation, SceneGraph::Camera3D &camera) {
    shader
            .setTransformationProjectionMatrix(camera.projectionMatrix() * transformation *
                                               Matrix4::rotationY(-90.0_degf) * Matrix4::rotationX(-90.0_degf))
            .draw(mesh);
}

} // namespace Magnum
