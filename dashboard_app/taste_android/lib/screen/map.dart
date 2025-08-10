import 'dart:math';
import 'package:animated_toggle_switch/animated_toggle_switch.dart';
import 'package:flutter/material.dart';
import 'package:logger/logger.dart';
import 'package:test_mqtt_client/components/utils/custom_colors.dart';
import 'package:test_mqtt_client/components/widgets/number_picker.dart';
import 'drawer_menu.dart';
import 'package:flutter/cupertino.dart';
import 'package:flutter/material.dart' hide Switch;
import 'package:firebase_database/firebase_database.dart';
import 'package:test_mqtt_client/components/widgets/battery_widget.dart';

class Mapping extends StatefulWidget {
  const Mapping({super.key});

  @override
  State<Mapping> createState() => _MapPage();
}

class _MapPage extends State<Mapping> {
  // double mapScale = 1.0;
  bool lightMode = false;
  double linearVel = 0.0;
  double angularVel = 0.0;
  final logger = Logger();
  Future<void> getLightMode() async {
    try {
      DataSnapshot lightSnapshot = await FirebaseDatabase.instance.ref("data/light").get();
      if (lightSnapshot.exists) {
        lightMode = lightSnapshot.value as bool;
      } else {
        logger.e("Light mode not found in Firebase.");
      }
    } catch (e) {
      logger.e("Error fetching light mode: $e");
    }
  }

  Future<void> getVelocitiesFromFirebase() async{
    DatabaseReference velRef = FirebaseDatabase.instance.ref();
    
    DataSnapshot linearSnapshot = await velRef.child("data/twist/linear_velocity").get();
    if (linearSnapshot.exists){
      setState(() {
        linearVel = double.tryParse(linearSnapshot.value.toString()) ?? 0.0;
      });
    }
    else {
      logger.w("Linear velocity not found in Firebase.");
    }
    DataSnapshot angularSnapshot = await velRef.child("data/twist/angular_velocity").get();
    if (angularSnapshot.exists){
      setState(() {
        angularVel = double.tryParse(angularSnapshot.value.toString()) ?? 0.0;
      });
    }
    else {
      logger.w("Angular velocity not found in Firebase.");
    }
  }

  Future<void> publishVelocity(String type, double value) async {
    final ref = FirebaseDatabase.instance.ref().child("data/twist/$type");
    await ref.set(value);
  }

  Future resetGoals() async{
    DatabaseReference delGoal = FirebaseDatabase.instance.ref();
    delGoal.child("data/number_of_goal").set(0);
  }

  void incrementLinearVel(){
    setState(() {
      if (linearVel < 1.0){
        linearVel += 0.1;
        linearVel = ((linearVel * 10).roundToDouble() / 10);
      }
    });
    publishVelocity("linear_velocity", linearVel);
  }
  void incrementAngularVel(){
    setState(() {
      if (angularVel < 2) {
        angularVel += 0.1;
        angularVel = ((angularVel * 10).roundToDouble() / 10);
      }
      logger.i("Linear: $linearVel, Angular: $angularVel");
    });
    publishVelocity("angular_velocity", angularVel);
  }
  void decrementLinearVel(){
    setState(() {
      if (linearVel > -1.0) {
        linearVel -= 0.1;
        linearVel = ((linearVel * 10).roundToDouble() / 10);
      }
      logger.i("Linear: $linearVel, Angular: $angularVel");
    });
    publishVelocity("linear_velocity", linearVel);
  }
  void decrementAngularVel(){
    setState(() {
      if (angularVel > -2.0) {
        angularVel -= 0.1;
        angularVel = ((angularVel * 10).roundToDouble() / 10);
      }
      logger.i("Linear: $linearVel, Angular: $angularVel");
    });
    publishVelocity("angular_velocity", angularVel);
  }

  void resetVel({bool withUI = true}) {
    if (withUI && mounted) {
      setState(() {
        linearVel = 0.0;
        angularVel = 0.0;
      });
    } else {
      linearVel = 0.0;
      angularVel = 0.0;
    }

    logger.i("Linear: $linearVel, Angular: $angularVel");
    publishVelocity("linear_velocity", 0.0);
    publishVelocity("angular_velocity", 0.0);
  }


  void resetSaveGoal() async{
    DatabaseReference saveGoalRef = FirebaseDatabase.instance.ref().child('data/save_goal');
    await saveGoalRef.set(false);
    DatabaseReference saveGoalIdRef = FirebaseDatabase.instance.ref().child('data/save_goal_id');
    await saveGoalIdRef.set(-1);
  }

  @override
  void initState() {
    super.initState();
    logger.i('Generate Map Screen');
    getVelocitiesFromFirebase();
    resetSaveGoal();
  }

  @override
  void dispose() {
    resetVel(withUI: false);
    // resetLightMode();
    super.dispose();
  }

  void _dialogConfirmDeleteGoals() {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
            title: const Center(
              child: Text('Notification!',
                  style: TextStyle(fontWeight: FontWeight.w500, fontSize: 28)),
            ),
            scrollable: true,
            content: Column(
              mainAxisSize: MainAxisSize.min,
              children: <Widget>[
                const Text.rich(
                  TextSpan(
                    children: [
                      TextSpan(
                        text: 'Are you sure want to',
                        style: TextStyle(fontSize: 20),
                      ),
                      TextSpan(
                        text: ' DELETE ALL GOALS',
                        style: TextStyle(
                          fontSize: 20,
                          color: Colors.red,
                        ),
                      ),
                    ],
                  ),
                ),
                SizedBox(height: MediaQuery.of(context).size.height * 0.02),
                Row(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    ElevatedButton(
                      onPressed: () {
                        Navigator.pop(context);
                      },
                      style: ElevatedButton.styleFrom(
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(8),
                        ),
                      ),
                      child: const Text(
                        'Cancel',
                        style: TextStyle(
                          fontSize: 15,
                          color: Colors.black,
                        ),
                      ),
                    ),
                    const SizedBox(width: 50),
                    ElevatedButton(
                      onPressed: () {
                        logger.i('Deleted all goals and reset home');
                        resetGoals();
                        Navigator.pop(context);
                      },
                      style: ElevatedButton.styleFrom(
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(8),
                        ),
                      ),
                      child: const Text(
                        ' Yes ',
                        style: TextStyle(
                          fontSize: 15,
                          color: Colors.red,
                        ),
                      ),
                    ),
                  ],
                )
              ],
            ));
      },
    );
  }

  void _showNumberPickerDialog(int maxValue) async{
    DatabaseReference saveGoalRef = FirebaseDatabase.instance.ref().child('data/');
    await saveGoalRef.child('save_goal').set(true);
    int selectedValue = 0;
    showDialog<int>(
      context: context,
      builder: (BuildContext context) {
        return Dialog(
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(20),
          ),
          elevation: 16,
          child: SizedBox(
            width: MediaQuery
                .of(context)
                .size
                .width * 0.35,
            child: Padding(
              padding: const EdgeInsets.all(20.0),
              child: Column(
                mainAxisSize: MainAxisSize.min,
                children: [
                  const Text(
                    'Select Goal',
                    style: TextStyle(
                        fontWeight: FontWeight.w500, fontSize: 28),
                  ),
                  SizedBox(height: MediaQuery
                      .of(context)
                      .size
                      .height * 0.015),
                  NumberPickerComponent(
                    maxValue: maxValue,
                    onChanged: (newValue) {
                      selectedValue = newValue;
                    },
                  ),
                  SizedBox(height: MediaQuery
                      .of(context)
                      .size
                      .height * 0.015),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      SizedBox(
                        width: 100,
                        height: 40,
                        child: ElevatedButton(
                          onPressed: () async {
                            await saveGoalRef.child('save_goal').set(false);
                            Navigator.of(context).pop();
                          },
                          style: ElevatedButton.styleFrom(
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                          child: const Text(
                            "Cancel",
                            style: TextStyle(
                              color: Colors.black,
                            ),
                          ),
                        ),
                      ),
                      const SizedBox(width: 30),
                      SizedBox(
                        width: 100,
                        height: 40,
                        child: ElevatedButton(
                          onPressed: () async {
                            if (maxValue <= 6 && selectedValue == maxValue) {
                              await saveGoalRef.child('number_of_goal').set(maxValue + 1);
                            }
                            
                            await saveGoalRef.child('save_goal_id').set(selectedValue);
                            await saveGoalRef.child('save_goal').set(false);
                            //if (!mounted) return;
                            Navigator.of(context).pop();
                          },
                          style: ElevatedButton.styleFrom(
                            shape: RoundedRectangleBorder(
                              borderRadius: BorderRadius.circular(8),
                            ),
                          ),
                          child: const Text(
                            "Save",
                            style: TextStyle(
                              color: Colors.black,
                            ),
                          ),
                        ),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ),
        );
      },
    );
  }


  @override
  Widget build(BuildContext context) {
    // double height = MediaQuery.of(context).size.height;
    // double width = MediaQuery.of(context).size.width;
    return Scaffold(
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
                        Icons.control_camera, // Icon bất kỳ (ở đây là sao)
                        color: Colors.white, // Màu của icon
                      ),
                      SizedBox(width: 8), // Khoảng cách giữa icon và chữ
                      Text(
                        'Robot Teleop',
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
      body: Center(
        child: Container(
          color: const Color.fromARGB(255, 237, 243, 250),
          child: Column(
            children: [
              SizedBox(
                height: MediaQuery.of(context).size.height * 0.05,
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.center,
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  const Text(
                    'Light mode: ',
                    style: TextStyle(fontSize: 20),
                  ),
                  SizedBox(
                    width: MediaQuery.of(context).size.width * 0.01,
                  ),
                  Container(
                    height: MediaQuery.of(context).size.height * 0.05,
                    color: const Color.fromARGB(255, 237, 243, 250),
                    child: Column(
                      children: [
                        SizedBox(
                          width: MediaQuery.of(context).size.height * 0.14,
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
                                    return FutureBuilder<DatabaseEvent>(
                                      future: FirebaseDatabase.instance.ref('data/mode').once(),
                                      builder: (context, modeSnapshot) {
                                        if (modeSnapshot.connectionState == ConnectionState.waiting) {
                                          return const CircularProgressIndicator();
                                        } else if (modeSnapshot.hasError) {
                                          return const Text("Can't load mode");
                                        } else if (!modeSnapshot.hasData ||
                                            modeSnapshot.data!.snapshot.value == null) {
                                          return const Text('Mode data is empty');
                                        } else {
                                          final mode_ = modeSnapshot.data!.snapshot.value.toString();
                                          final isPatrol = mode_ == 'PATROL';
                                          getLightMode();
                                          return IgnorePointer(
                                            ignoring: mode_ == 'PATROL',
                                            child: AnimatedToggleSwitch<bool>.dual(
                                              current: lightMode,
                                              first: false,
                                              second: true,
                                              spacing: 45.0,
                                              animationDuration:
                                              const Duration(milliseconds: 400),
                                              style: ToggleStyle(
                                                borderColor: Colors.transparent,
                                                indicatorColor: Colors.white,
                                                backgroundColor: isPatrol ? Colors.grey : Colors.amber,
                                              ),
                                              customStyleBuilder: (context, local, global) =>
                                                  ToggleStyle(
                                                    backgroundGradient: LinearGradient(
                                                      colors: isPatrol ? [Colors.grey, Colors.grey] : [Colors.green, Colors.black],
                                                      stops: [
                                                        global.position - (1 - 2 * max(0,global.position - 0.5)) * 0.5,
                                                        global.position + max(0,2 * (global.position - 0.5)) * 0.5,
                                                      ],
                                                    ),
                                                  ),
                                              borderWidth: 3.0,
                                              height: 40.0,
                                              loadingIconBuilder: (context, global) =>
                                                  CupertinoActivityIndicator(
                                                    color: Color.lerp(Colors.black, Colors.green,
                                                        global.position),
                                                  ),
                                              onChanged: isPatrol ? null : ((bool newValue) async{
                                                setState(() => lightMode = newValue);
                                                logger.i("mode_: $mode_");  // để kiểm tra
                                                DatabaseReference modeRef = FirebaseDatabase.instance.ref().child('data/light');
                                                await modeRef.set(newValue);
                                                return Future<dynamic>.delayed(
                                                    const Duration(seconds: 2));
                                              }),
                                              iconBuilder: (bool value) => value
                                                  ? Icon(Icons.sunny,
                                                  color: isPatrol ? Colors.grey: Colors.yellow, size: 25.0)
                                                  : Icon(
                                                  Icons.dark_mode,
                                                  color: isPatrol ? Colors.grey: Colors.black,
                                                  size: 32.0),
                                              textBuilder: (bool value) => Center(
                                                child: Text(
                                                  value ? 'ON' : 'OFF',
                                                  style: const TextStyle(
                                                    color: Colors.white,
                                                    fontSize: 20.0,
                                                    fontWeight: FontWeight.w600,
                                                  ),
                                                ),
                                              ),
                                            ),
                                          );
                                        }
                                      }
                                    );
                                  }
                                  //}
                                }
                              },
                            )
                          )
                        ),
                      ]
                    )
                  )
                ]
              ),
              const SizedBox(height: 20),
              Text(
                  'Linear velocity: $linearVel',
                  style: const TextStyle(fontSize: 20),
              ),
              Text(
                  'Angular velocity: $angularVel',
                  style: const TextStyle(fontSize: 20),
              ),              
              const SizedBox(
                height: 20,
                width: 20,
              ),
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
                    } else {
                      final mode_ = snapshot.data!.snapshot.value.toString();
                      final isPatrol = mode_ == 'PATROL';
                        return
                          Center(
                            child: Container(
                              height: MediaQuery.of(context).size.height * 0.4,
                              color: const Color.fromARGB(255, 237, 243, 250),
                              child: Column(
                                mainAxisSize: MainAxisSize.min,
                                children: [
                                  ElevatedButton(
                                    onPressed: isPatrol ? null : incrementLinearVel,
                                    style: ElevatedButton.styleFrom(
                                        padding: const EdgeInsets.all(0),
                                        elevation: 10,
                                        shadowColor: Colors.black.withAlpha(102),
                                        backgroundColor: isPatrol ? Colors.grey[600] : null,
                                        minimumSize: const Size(100, 70)
                                    ),
                                    child: const Icon(
                                      Icons.arrow_drop_up,
                                      color: Colors.black,
                                      size: 80,
                                    ),
                                  ),
                                  const SizedBox(height: 10),
                                  Center(
                                    child: Row(
                                      mainAxisAlignment: MainAxisAlignment.center,
                                      children: [
                                        ElevatedButton(
                                            onPressed: isPatrol ? null : incrementAngularVel,
                                            style: ElevatedButton.styleFrom(
                                                padding: const EdgeInsets.all(0),
                                                elevation: 10,
                                                shadowColor: Colors.black.withAlpha(102),
                                                backgroundColor: isPatrol ? Colors.grey[600] : null,
                                                minimumSize: const Size(100, 70)
                                            ),
                                            child: const Icon(
                                              Icons.arrow_left,
                                              color: Colors.black,
                                              size: 80,
                                            )
                                        ),
                                        const SizedBox(width: 10),
                                        ElevatedButton(
                                            onPressed: isPatrol ? null : resetVel,
                                            style: ElevatedButton.styleFrom(
                                                padding: const EdgeInsets.all(15),
                                                elevation: 10,
                                                shadowColor: Colors.black.withAlpha(102),
                                                backgroundColor: isPatrol ? Colors.grey[600] : null,
                                                minimumSize: const Size(100, 70)
                                            ),
                                            child: const Icon(
                                              Icons.circle,
                                              color: Colors.red,
                                              size: 60,
                                            )
                                        ),
                                        const SizedBox(width: 10),
                                        ElevatedButton(
                                            onPressed: isPatrol ? null : decrementAngularVel,
                                            style: ElevatedButton.styleFrom(
                                                padding: const EdgeInsets.all(0),
                                                elevation: 10,
                                                shadowColor: Colors.black.withAlpha(102),
                                                backgroundColor: isPatrol ? Colors.grey[600] : null,
                                                minimumSize: const Size(100, 70)
                                            ),
                                            child: const Icon(
                                              Icons.arrow_right,
                                              color: Colors.black,
                                              size: 80,
                                            )
                                        )
                                      ],
                                    ),
                                  ),
                                  const SizedBox(height: 10),
                                  ElevatedButton(
                                      onPressed: isPatrol ? null : decrementLinearVel,
                                      style: ElevatedButton.styleFrom(
                                          padding: const EdgeInsets.all(0),
                                          elevation: 10,
                                          shadowColor:  Colors.black.withAlpha(102),
                                          backgroundColor: isPatrol ? Colors.grey[600] : null,
                                          minimumSize: const Size(100, 70)
                                      ),
                                      child: const Icon(
                                        Icons.arrow_drop_down,
                                        color: Colors.black,
                                        size: 80,
                                      )
                                  )
                                ],
                              ),
                            ),
                          );
                    }
                  }

              ),
              Center(
                child: Container(
                  height: MediaQuery.of(context).size.height * 0.2,
                  color: const Color.fromARGB(255, 237, 243, 250),
                  child:
                  Row(
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
                      StreamBuilder(
                          stream: FirebaseDatabase.instance.ref().child('data/ready').onValue,
                          builder: (context, snapshot) {
                            if (snapshot.connectionState == ConnectionState.waiting) {
                              return const CircularProgressIndicator(); // Chờ khi đang tải dữ liệu
                            } else if (snapshot.hasError) {
                              return const Text('Error'); // Hiển thị lỗi nếu có lỗi
                            } else if (!snapshot.hasData || snapshot.data!.snapshot.value == null) {
                              return const Text('Not found data'); // Trường hợp không có dữ liệu pin
                            }
                            else{
                              // final ready = snapshot.data!.snapshot.value as bool;
                              // if (ready == false){
                              //   return const CircularProgressIndicator();
                              // }
                              //else{
                                return
                                  StreamBuilder(
                                    stream: FirebaseDatabase.instance.ref().child('data/mode').onValue,
                                    builder: (context, snapshot){
                                      if (snapshot.connectionState == ConnectionState.waiting) {
                                        return const CircularProgressIndicator(); // Chờ khi đang tải dữ liệu
                                      } else if (snapshot.hasError) {
                                        return const Text('Error'); // Hiển thị lỗi nếu có lỗi
                                      } else if (!snapshot.hasData || snapshot.data!.snapshot.value == null) {
                                        return const Text('Not found data'); // Trường hợp không có dữ liệu pin
                                      }
                                      else{
                                        final mode_ = snapshot.data!.snapshot.value?.toString();
                                        if (mode_ == null) {
                                          return const Text('Mode is not available');
                                        }
                                        else if (mode_ == 'PATROL'){
                                          return
                                            Column(
                                              mainAxisAlignment: MainAxisAlignment.center,
                                              children: [
                                                buildActionButton(
                                                  label: 'Save Goal', 
                                                  onPressed: null
                                                ),

                                                const SizedBox(
                                                  height: 10,
                                                ),

                                                buildActionButton(
                                                  label: 'Delete Goals', 
                                                  onPressed: null
                                                ),
                                              ],
                                            );
                                        }
                                        else if (mode_ == 'NORMAL'){
                                          return
                                            Column(
                                              mainAxisAlignment: MainAxisAlignment.center,
                                              children: [
                                                StreamBuilder(
                                                    stream: FirebaseDatabase.instance.ref().child('data/number_of_goal').onValue,
                                                    builder: (context, snapshot){
                                                      if (snapshot.connectionState == ConnectionState.waiting) {
                                                        return const CircularProgressIndicator(); // Chờ khi đang tải dữ liệu
                                                      } else if (snapshot.hasError) {
                                                        return const Text('Error'); // Hiển thị lỗi nếu có lỗi
                                                      } else if (!snapshot.hasData || snapshot.data!.snapshot.value == null) {
                                                        return const Text('Not found data'); // Trường hợp không có dữ liệu pin
                                                      }
                                                      else{
                                                        final numberOfgoal = snapshot.data!.snapshot.value as int;
                                                        if (numberOfgoal >= 0 && numberOfgoal <= 7) {
                                                          return
                                                            buildActionButton(
                                                              label: 'Save Goal', 
                                                              onPressed: () { _showNumberPickerDialog(numberOfgoal);},
                                                            );
                                                        }
                                                        else{
                                                          logger.e('Something has gone wrong');
                                                          return const CircularProgressIndicator();
                                                        }
                                                      }
                                                    }
                                                ),
                                                const SizedBox(
                                                  height: 10,
                                                ),
                                                buildActionButton(
                                                  label: 'Delete Goals', 
                                                  onPressed: () { _dialogConfirmDeleteGoals();},
                                                ),
                                              ],
                                            );
                                        }
                                        else{
                                          return const CircularProgressIndicator();
                                        }
                                        // else{
                                        //   print('Something has gone wrong');
                                        //   return const CircularProgressIndicator();
                                        // }
                                      }
                                    }
                                  );
                              }
                            }
                          //}
                          )
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
      drawer: const DrawerMenu(),
    );
  }
}

Widget buildActionButton({
  required String label,
  required VoidCallback? onPressed,
}) {
  return ElevatedButton(
    onPressed: onPressed,
    style: ElevatedButton.styleFrom(
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(8),
      ),
      backgroundColor: onPressed == null ? Colors.grey[600] : label == 'Save Goal' ? Colors.blue[300] : Colors.red[300],
      elevation: 10,
      shadowColor: onPressed == null ? null : Colors.black.withAlpha(102),
    ),
    child: Text(
      label,
      style: const TextStyle(color: Colors.black),
    ),
  );
}