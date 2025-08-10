import 'package:flutter/material.dart';
import 'package:flutter_svg/flutter_svg.dart';
import 'package:logger/logger.dart';
import 'package:test_mqtt_client/components/utils/custom_colors.dart';
import 'package:test_mqtt_client/components/widgets/battery_widget.dart';
import 'package:test_mqtt_client/components/widgets/current_goal_widget.dart';
// import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_database/firebase_database.dart';
import 'drawer_menu.dart';
import 'package:test_mqtt_client/components/widgets/reset_vel.dart';

class Home extends StatefulWidget {
  const Home({super.key});

  @override
  State<Home> createState() => _HomeState();
}

class _HomeState extends State<Home> {
  int selectedGoalId = -1;
  bool isRequestPending = false;
  final logger = Logger();
  @override
  void initState() {
    super.initState();
    logger.i('Generate Home Screen');
    resetLinearVel();
    resetAngularVel();
  }

  @override
  void dispose() {
    super.dispose();
  }
  
  void _sendGoalToFirebase(int goalNumber) {
    // Khởi tạo Firebase Realtime Database
    final databaseRef = FirebaseDatabase.instance.ref();
    // Cập nhật giá trị navigate_to_goal lên Firebase
    databaseRef.child('data/navigate_to_goal').set(goalNumber).then((_) {
      logger.i("Goal number $goalNumber sent to Firebase.");
    }).catchError((error) {
      logger.e("Error sending data to Firebase: $error");
    });
  }

  @override
  Widget build(BuildContext context) {
    return  Scaffold(
      backgroundColor:  CustomColors.backgroundColors,
      appBar: PreferredSize(
        preferredSize: Size.fromHeight(MediaQuery.of(context).size.height * 0.068),
        child: AppBar(
          backgroundColor: CustomColors.backgroundColors,
          title:
          Center(
            child: Container(
              padding: const EdgeInsets.all(8),
              color: Colors.blue[900], // Nền màu xanh đậm
              child: const Center(
                child: Row(
                    mainAxisSize: MainAxisSize.min,
                    children: [
                      Icon(
                        Icons.ads_click_rounded, // Icon bất kỳ (ở đây là sao)
                        color: Colors.white, // Màu của icon
                      ),
                      SizedBox(width: 8), // Khoảng cách giữa icon và chữ
                      Text(
                        'Goal Navigation',
                        style: TextStyle(
                          color: Colors.white, // Màu chữ trắng
                          fontSize: 16, // Kích thước chữ
                        ),
                      ),
                    ]
                ),
              ),
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
            margin: const EdgeInsets.all(30),
            height: MediaQuery.of(context).size.height * 0.5,
            color: const Color.fromARGB(255, 237, 243, 250),
            child: SingleChildScrollView(
              child: Column(
                children: [
                  StreamBuilder(
                    stream: FirebaseDatabase.instance.ref('data/ready').onValue,
                    builder: (context, snapshot) {
                      if (snapshot.connectionState == ConnectionState.waiting) {
                        return const CircularProgressIndicator();
                      } else if (snapshot.hasError) {
                        return const Text("Can't load goal state data");
                      } else if (!snapshot.hasData || snapshot.data!.snapshot.value == null) {
                        return const Text('No data available');
                      } else {
                        final isready = snapshot.data!.snapshot.value as bool;
                        if (isready == false){
                          return const CircularProgressIndicator();
                        }
                        else{
                          return
                            StreamBuilder(
                                stream: FirebaseDatabase.instance.ref('data/mode').onValue,
                                builder: (context, snapshot){
                                  if (snapshot.connectionState ==
                                      ConnectionState.waiting) {
                                    return const CircularProgressIndicator();
                                  } else if (snapshot.hasError) {
                                    return const Text("Can't load goal state data");
                                  } else if (!snapshot.hasData ||
                                      snapshot.data!.snapshot.value == null) {
                                    return const Text('No data available');
                                  } else{
                                    final mode_ = snapshot.data!.snapshot.value as String;
                                    if (mode_ == 'PATROL'){
                                      return
                                        Container(
                                          width: MediaQuery.of(context).size.width * 0.8,
                                          padding: const EdgeInsets.all(8),
                                          color: const Color.fromARGB(255, 237, 243, 250),
                                          child: Column(
                                            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                                            children: [
                                              for (int i = 0; i < 7; i++)
                                                ElevatedButton(
                                                  onPressed: null,
                                                  style: ElevatedButton.styleFrom(
                                                    shape: RoundedRectangleBorder(
                                                      borderRadius: BorderRadius.circular(12.0),
                                                    ),
                                                    backgroundColor: Colors.grey[600],
                                                    minimumSize: const Size(250, 40),
                                                  ),
                                                  child: Text(i == 0 ? 'Home' : 'Goai $i', style: const TextStyle(color: Colors.black, fontSize: 20)),
                                                ),
                                            ],
                                          ),
                                        );
                                    }
                                    else if (mode_ == 'NORMAL'){ 
                                      return
                                        StreamBuilder(
                                            stream: FirebaseDatabase.instance.ref('data/number_of_goal').onValue,
                                            builder: (context, snapshot){
                                              if (snapshot.connectionState ==
                                                  ConnectionState.waiting) {
                                                return const CircularProgressIndicator();
                                              } else if (snapshot.hasError) {
                                                return const Text("Can't load goal state data");
                                              } else if (!snapshot.hasData ||
                                                  snapshot.data!.snapshot.value == null) {
                                                return const Text('No data available');
                                              }else{
                                                final numberOfgoal = snapshot.data!.snapshot.value;
                                                if (numberOfgoal == null) {
                                                  return const CircularProgressIndicator(); // Đang tải dữ liệu
                                                }

                                                if (numberOfgoal is! int || numberOfgoal < 0) {
                                                  return const Text("Lỗi: Số goal không hợp lệ");
                                                }

                                                return Container(
                                                  width: MediaQuery.of(context).size.width * 0.8,
                                                  padding: const EdgeInsets.all(8),
                                                  color: const Color.fromARGB(255, 237, 243, 250),
                                                  child: Column(
                                                    mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                                                    children: [
                                                      for (int j = 0; j < 7; j++)
                                                        ElevatedButton(
                                                          onPressed: numberOfgoal > j
                                                              ? () {
                                                                  j == 0
                                                                      ? logger.i('Home button pressed')
                                                                      : logger.i('Goal $j button pressed');
                                                                  _sendGoalToFirebase(j);
                                                                }
                                                              : null,
                                                          style: ElevatedButton.styleFrom(
                                                            shape: RoundedRectangleBorder(
                                                              borderRadius: BorderRadius.circular(12.0),
                                                            ),
                                                            backgroundColor: Colors.grey[600],
                                                            minimumSize: const Size(250, 40),
                                                          ),
                                                          child: Text(
                                                            j == 0 ? 'Home' : 'Goal $j',
                                                            style: const TextStyle(color: Colors.black, fontSize: 20),
                                                          ),
                                                        ),
                                                    ],
                                                  ),
                                                );
                                              }
                                            }
                                        );
                                    }
                                    else{
                                      logger.e('Something has gone wrong');
                                      return const CircularProgressIndicator();
                                    }
                                  }
                                }
                            );
                        }
                      }
                    },
                  ),
                ],
              ),
            ),
          ),

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
                SizedBox(width: MediaQuery.of(context).size.width * 0.3),

                Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    SizedBox(
                      width: MediaQuery.of(context).size.width * 0.3,
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
      drawer: const DrawerMenu(),
    );
  }
}
