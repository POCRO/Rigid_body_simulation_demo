#include <Tudat/SimulationSetup/tudatSimulationHeader.h>
#include <Tudat/Astrodynamics/Aerodynamics/nrlmsise00Atmosphere.h>


int main()
{
    using namespace tudat;
    using namespace tudat::simulation_setup;
    using namespace tudat::basic_astrodynamics;
    using namespace tudat::numerical_integrators;
    using namespace tudat::propagators;
    using namespace tudat::input_output;

    // Step 1: 设置大气模型和地球环境
    std::shared_ptr<Body> earth = std::make_shared<Body>();
    earth->setAtmosphereModel(createAtmosphereModel(
        tudat::atmosphere_models::nrlmsise00Atmosphere, {}));
    earth->setShapeModel(createGeoidModel());

    SystemOfBodies bodies;
    bodies.createEmptyBody("Earth");
    bodies.addBody("Earth", earth);

    // Step 2: 定义航天器
    std::shared_ptr<Body> spacecraft = std::make_shared<Body>();
    spacecraft->setAerodynamicCoefficientInterface(createAerodynamicCoefficientInterface(
        tudat::aerodynamics::constantAerodynamicCoefficientSettings(
            Eigen::Vector3d(2.2, 0.0, 0.0), 3.0))); // Cd = 2.2, Area = 3m²
    bodies.addBody("Spacecraft", spacecraft);

    // Step 3: 初始状态设置（高度 120 km，速度 7.5 km/s）
    Eigen::Vector6d initialState;
    initialState << 6578e3, 0.0, 0.0, 0.0, 7.5e3, 0.0;

    // Step 4: 设置仿真参数
    double simulationStartEpoch = 0.0;              // 开始时间
    double simulationEndEpoch = 300.0;             // 仿真时长
    double fixedStepSize = 0.1;                    // 积分步长

    std::shared_ptr<PropagationTerminationSettings> terminationSettings =
        std::make_shared<PropagationTerminationSettings>(
            tudat::propagation_setup::PropagationTerminationSettings(
                tudat::propagation_setup::TerminationSettings(
                    tudat::propagation_setup::propagation_termination::time_termination,
                    simulationEndEpoch)));

    // Step 5: 定义加速度
    SelectedAccelerationMap accelerationSettingsMap;
    accelerationSettingsMap["Spacecraft"]["Earth"].push_back(
        std::make_shared<AccelerationSettings>(basic_astrodynamics::aerodynamic));
    accelerationSettingsMap["Spacecraft"]["Earth"].push_back(
        std::make_shared<AccelerationSettings>(basic_astrodynamics::central_gravity));

    AccelerationMap accelerationModelMap = createAccelerationModelsMap(
        bodies, accelerationSettingsMap, {"Spacecraft"}, {"Earth"});

    // Step 6: 配置数值积分器
    std::shared_ptr<IntegratorSettings<>> integratorSettings =
        std::make_shared<IntegratorSettings<>>(
            numerical_integrators::rungeKutta4, simulationStartEpoch, fixedStepSize);

    // Step 7: 创建动力学仿真器
    TranslationalStatePropagatorSettings<> propagatorSettings(
        {"Earth"}, accelerationModelMap, {"Spacecraft"}, initialState, simulationStartEpoch,
        terminationSettings);

    SingleArcDynamicsSimulator<> dynamicsSimulator(
        bodies, integratorSettings, std::make_shared<PropagatorSettings<>>(
                                         propagatorSettings));

    // Step 8: 提取仿真结果
    std::map<double, Eigen::VectorXd> simulationResults =
        dynamicsSimulator.getEquationsOfMotionNumericalSolution();

    // Step 9: 保存结果到文件
    input_output::writeDataMapToTextFile(simulationResults, "ReentrySimulationResults.dat", "./");

    return 0;
}
