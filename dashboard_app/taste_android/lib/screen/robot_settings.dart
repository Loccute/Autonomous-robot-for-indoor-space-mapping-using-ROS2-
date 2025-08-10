import 'package:animated_toggle_switch/animated_toggle_switch.dart';
import 'package:flutter/material.dart' hide Switch;
import 'package:flutter_svg/flutter_svg.dart';
// import 'package:provider/provider.dart';
import 'package:flutter/cupertino.dart';
import 'package:logger/web.dart';
import 'package:test_mqtt_client/components/utils/custom_colors.dart';
import 'drawer_menu.dart';
import 'dart:math';
import 'package:firebase_database/firebase_database.dart';
import 'package:test_mqtt_client/components/widgets/battery_widget.dart';
import 'package:test_mqtt_client/components/widgets/current_goal_widget.dart';
import 'package:test_mqtt_client/components/widgets/reset_vel.dart';

class RobotSetting extends StatefulWidget {
  const RobotSetting({super.key});

  @override
  State<RobotSetting> createState() => _RobotSettingState();
}
class _RobotSettingState extends State<RobotSetting> {
  bool isAutonomous = false;
  final logger = Logger();
  @override
  void initState() {
    super.initState();
    resetLinearVel();
    resetAngularVel();
    logger.i('Generate Setting Screen');
    FirebaseDatabase.instance.ref().child('data/mode').onValue.listen((event){
      final mode = event.snapshot.value;
      if (mode != null){
        setState(() {
          isAutonomous = mode == 'NORMAL';
        });
      }
    }
    );
  }

  @override
  void dispose() {
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor:  CustomColors.backgroundColors,
      appBar: PreferredSize(
        preferredSize: Size.fromHeight(MediaQuery.of(context).size.height * 0.068),
        child: AppBar(
          backgroundColor: CustomColors.backgroundColors,
          title:
          Center(
            child: Column(
              children: [
                Container(
                  padding: const EdgeInsets.all(8),
                  color: Colors.blue[900], // Nền màu xanh đậm
                  child: const Center(
                    child: Row(
                        mainAxisSize: MainAxisSize.min,
                        children: [
                          Icon(
                            Icons.smart_toy_outlined, // Icon bất kỳ (ở đây là sao)
                            color: Colors.white, // Màu của icon
                          ),
                          SizedBox(width: 8), // Khoảng cách giữa icon và chữ
                          Text(
                            'Mode Setting',
                            style: TextStyle(
                              color: Colors.white, // Màu chữ trắng
                              fontSize: 16, // Kích thước chữ
                            ),
                          ),
                        ]
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
      body: Column(
        children: [
          Container(
            height: MediaQuery.of(context).size.height * 0.003,
            width: MediaQuery.of(context).size.width * 0.8,
            color: Colors.grey,
            margin: EdgeInsets.symmetric(
                vertical: MediaQuery.of(context).size.height * 0.01),
          ),
          Container(
            height: MediaQuery.of(context).size.height * 0.3,
          ),
          Container(
            height: MediaQuery.of(context).size.height * 0.5,
            color: const Color.fromARGB(255, 237, 243, 250),
            child: Column(
              children: [
                Column(
                  children: [
                    SizedBox(
                      width: MediaQuery.of(context).size.height * 0.3,
                    ),
                    const Text(
                      'Switch Mode: ',
                      style: TextStyle(
                        color: Colors.black,
                        fontSize: 20.0,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                    SizedBox(
                      width: MediaQuery.of(context).size.width * 0.15,
                    ),
                    SizedBox(
                      width: MediaQuery.of(context).size.height * 0.25,
                      child: SizedBox(
                        child: StreamBuilder(
                          stream: FirebaseDatabase.instance.ref().child('data/ready').onValue,
                          builder: (context, snapshot) {
                            if (snapshot.connectionState ==
                                ConnectionState.waiting) {
                              return const CircularProgressIndicator();
                            } else if (snapshot.hasError) {
                              return const Text(
                                  "Can't load operation mode data");
                            } else if (!snapshot.hasData ||
                                snapshot.data!.snapshot.value == null) {
                              return const Text('Operation mode data is empty');
                            } else {
                              final isready = snapshot.data!.snapshot.value as bool;
                              if(isready == false){
                                return const CircularProgressIndicator();
                              }
                              else{
                                return AnimatedToggleSwitch<bool>.dual(
                                  current: isAutonomous,
                                  first: false,
                                  second: true,
                                  spacing: 45.0,
                                  animationDuration:
                                  const Duration(milliseconds: 400),
                                  style: const ToggleStyle(
                                    borderColor: Colors.transparent,
                                    indicatorColor: Colors.white,
                                    backgroundColor: Colors.amber,
                                  ),
                                  customStyleBuilder: (context, local, global) =>
                                      ToggleStyle(
                                        backgroundGradient: LinearGradient(
                                          colors: const [Colors.green, Colors.red],
                                          stops: [
                                            global.position - (1 - 2 * max(0,global.position - 0.5)) * 0.5,
                                            global.position + max(0,2 * (global.position - 0.5)) * 0.5,
                                          ],
                                        ),
                                      ),
                                  borderWidth: 6.0,
                                  height: 60.0,
                                  loadingIconBuilder: (context, global) =>
                                      CupertinoActivityIndicator(
                                        color: Color.lerp(Colors.red, Colors.green,
                                            global.position),
                                      ),
                                  onChanged: (bool newValue) async{
                                    setState(() => isAutonomous = newValue);
                                    final selectedMode =
                                    newValue ? 'NORMAL' : 'PATROL';
                                    DatabaseReference modeRef = FirebaseDatabase.instance.ref().child('data/mode');
                                    await modeRef.set(selectedMode);
                                    return Future<dynamic>.delayed(
                                        const Duration(seconds: 2));
                                  },
                                  iconBuilder: (bool value) => value
                                      ? const Icon(Icons.delivery_dining,
                                      color: Colors.green, size: 32.0)
                                      : const Icon(
                                      Icons.autorenew,
                                      color: Colors.red,
                                      size: 32.0),
                                  textBuilder: (bool value) => Center(
                                    child: Text(
                                      value ? 'NORMAL' : 'PATROL',
                                      style: const TextStyle(
                                        color: Colors.white,
                                        fontSize: 20.0,
                                        fontWeight: FontWeight.w600,
                                      ),
                                    ),
                                  ),
                                );
                              }
                            }
                          },
                        ),
                      ),
                    ),
                  ],
                ),
                SizedBox(height: MediaQuery.of(context).size.height * 0.1),
                Container(
                  height: MediaQuery.of(context).size.height * 0.2,
                  color: CustomColors.backgroundColors,
                  child: Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      StreamBuilder(
                          stream: FirebaseDatabase.instance.ref().child('data/battery').onValue,
                          builder: (context, snapshot) {
                            if (snapshot.connectionState == ConnectionState.waiting) {
                              return const CircularProgressIndicator(); // Chờ khi đang tải dữ liệu
                            } else if (snapshot.hasError) {
                              return const Text(
                                  'Lỗi khi lấy dữ liệu pin'); // Hiển thị lỗi nếu có lỗi
                            } else if (!snapshot.hasData ||
                                snapshot.data!.snapshot.value == null) {
                              return const Text(
                                  'Không có dữ liệu pin'); // Trường hợp không có dữ liệu pin
                            } else {
                              final batteryLevel = snapshot.data!.snapshot
                                  .value as int; // Lấy dữ liệu pin dưới dạng double
                              return SizedBox(
                                height: MediaQuery
                                    .of(context)
                                    .size
                                    .height * 0.15,
                                child: BatteryWidget(
                                    batteryLevel: batteryLevel), // Hiển thị widget pin
                              );
                            }
                          }
                      ),
                      SizedBox(width: MediaQuery.of(context).size.width * 0.2),

                      Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          SizedBox(
                            child: StreamBuilder(
                              stream: FirebaseDatabase.instance.ref().child('data/goal/state').onValue,
                              builder: (context, snapshot) {
                                if (snapshot.connectionState == ConnectionState.waiting) {
                                  return const CircularProgressIndicator();
                                } else if (snapshot.hasError) {
                                  return const Text('Error loading goal state');
                                } else if (!snapshot.hasData || snapshot.data!.snapshot.value == null) {
                                  return const Text('No goal state data');
                                } else {
                                  String goalState = snapshot.data!.snapshot.value as String;
                                  return SizedBox(
                                    width: 150,
                                    child: Column(
                                      children: [
                                        goalState == 'SUCCESS' ?
                                        (SvgPicture.asset(
                                          'assets/goal_state/success.svg',
                                          height: 100,
                                        )) 
                                        : goalState == 'FAILURE' ?
                                        (SvgPicture.asset(
                                          'assets/goal_state/fail.svg',
                                          height: 100,
                                        ))
                                        : goalState == 'PROCESSING' ?
                                        (SvgPicture.asset(
                                          'assets/goal_state/processing.svg',
                                          height: 100,
                                        ))
                                        : (const CircularProgressIndicator()),
                                      ],
                                    ),
                                  );
                                }
                              },
                            ),
                          ),
                          SizedBox(
                            // height: MediaQuery.of(context).size.height * 0.15,
                            child: StreamBuilder(
                              stream: FirebaseDatabase.instance
                                  .ref('data/goal') // Đường dẫn đến mục 'goal' trên Firebase Realtime Database
                                  .onValue, // Lắng nghe thay đổi của 'goal'
                              builder: (context, snapshot) {
                                if (snapshot.connectionState == ConnectionState.waiting) {
                                  return const CircularProgressIndicator();
                                } else if (snapshot.hasError) {
                                  return const Text("Can't load goal state data");
                                } else if (!snapshot.hasData || snapshot.data!.snapshot.value == null) {
                                  return const Text('No data available');
                                } else {
                                  // Lấy dữ liệu từ snapshot Firebase
                                  final data = snapshot.data!.snapshot.value as Map<dynamic, dynamic>;

                                  final currentGoalId = (data['current_id'] as num?)?.toInt(); // Trường 'current_id' từ Firebase
                                  final estimatedTime = (data['estimate_time'] as num?)?.toDouble(); // Trường 'estimate_time' từ Firebase

                                  // Kiểm tra dữ liệu có hợp lệ không
                                  if (currentGoalId == null || estimatedTime == null) {
                                    return const Text('Data is incomplete');
                                  }

                                  // Hiển thị dữ liệu trong widget
                                  return CurrentGoalWidget(
                                    currentGoalId: currentGoalId,
                                    estimatedTimeRemaining: estimatedTime,
                                  );
                                }
                              },
                            ),
                          ),
                        ],
                      ),
                    ],
                  ),
                ),
              ],
            ),
          ),
        ],
      ),
      drawer: const DrawerMenu(),
    );
  }
}
