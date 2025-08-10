import 'package:flutter/material.dart';
import 'package:test_mqtt_client/screen/robot_settings.dart';
import 'package:test_mqtt_client/screen/splash.dart';
import 'screen/home.dart';
import 'screen/map.dart';
import 'screen/login.dart';
import 'package:firebase_core/firebase_core.dart';
import 'firebase_options.dart';
import 'package:firebase_database/firebase_database.dart';
import 'screen/register.dart';
import 'package:logger/logger.dart';


final GlobalKey<NavigatorState> navigatorKey = GlobalKey<NavigatorState>();

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await Firebase.initializeApp(
    options: DefaultFirebaseOptions.currentPlatform,
  );
  runApp(const Main());
}

class Main extends StatefulWidget {
  const Main({super.key});

  @override
  State<Main> createState() => _MainState();
}

class _MainState extends State<Main> with WidgetsBindingObserver {
  final DatabaseReference usingRef = FirebaseDatabase.instance.ref("data/users/using");
  final logger = Logger();
  
  @override
  void initState() {
    super.initState();
    // Add observer to listen for app lifecycle changes
    WidgetsBinding.instance.addObserver(this);
  }

  @override
  void dispose() {
    // Remove observer when the widget is disposed
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  Future<void> _saveLastActiveTime() async {
    DatabaseReference userRef = FirebaseDatabase.instance.ref("data/users/lastActiveTime");
    await userRef.set(DateTime.now().millisecondsSinceEpoch);
  }

  // Override this method to handle lifecycle changes
  @override
  void didChangeAppLifecycleState(AppLifecycleState state) async{
    super.didChangeAppLifecycleState(state);
    (state == AppLifecycleState.paused) ? logger.i("state 1") : (state == AppLifecycleState.resumed) ? logger.i("state 2") : (state == AppLifecycleState.inactive) ? logger.i("state 3") : (state == AppLifecycleState.detached) ? logger.i("state 4") : (state == AppLifecycleState.hidden) ? logger.i("state 5") : logger.i("No state available");
    // Handle lifecycle events
    
    bool using = false;
    try {
      final snapshot = await usingRef.get();
      using = snapshot.value as bool? ?? false;
    } catch (e) {
      logger.e("Lỗi khi lấy giá trị 'using': $e");
    }

    if (state == AppLifecycleState.paused) {
      if (using == true) await _saveLastActiveTime(); // Ghi thời gian
      logger.i("App is in the background");
      // You can add actions here like saving data or logging out users
    } else if (state == AppLifecycleState.resumed) {
      if (using == true) await _saveLastActiveTime(); // Ghi thời gian
      logger.i("App is in the foreground");
      // Perform actions when the app comes to the foreground (e.g., refreshing data)
    }
    else if (state == AppLifecycleState.inactive) {
      if (using == true) await _saveLastActiveTime(); // Ghi thời gian
      logger.i("App is inactive");
      // Handle transitions, such as between foreground and background
    } else if (state == AppLifecycleState.detached) {
      if (using == true) await _saveLastActiveTime(); // Ghi thời gian
      logger.i("App is detached");
      // Handle the rare case when the app is detached from the view
    }
  }

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      navigatorKey: navigatorKey,
      theme: ThemeData(
        appBarTheme: const AppBarTheme(),
      ),
      debugShowCheckedModeBanner: false,
      initialRoute: '/splash',
      routes: {
        '/splash': (context) => const SplashScreen(),
        '/login': (context) => LoginPage(),
        '/register': (context) => const RegisterPage(),
        '/home': (context) => const Home(),
        '/mapping': (context) => const Mapping(),
        '/setting': (context) => const RobotSetting(),
      },
    );
  }
}
