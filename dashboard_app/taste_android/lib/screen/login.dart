import 'package:flutter/material.dart';
// import 'package:firebase_core/firebase_core.dart';
import 'package:firebase_database/firebase_database.dart';
import 'package:logger/logger.dart';
import 'package:flutter/services.dart'; // Cho platform channel


class LoginPage extends StatelessWidget {
  // Step 1: Create TextEditingController instances for username and password
  final TextEditingController _usernameController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();
  static const platform = MethodChannel('com.example.test_mqtt_client/service'); // đổi đúng package
  final logger = Logger();
  LoginPage({super.key});

  @override
  Widget build(BuildContext context) {
    return SafeArea(
      child: Scaffold(
        body: Container(
          margin: const EdgeInsets.all(10),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.spaceEvenly,
            children: [
              _header(),
              _inputField(context),
            ],
          ),
        ),
      ),
    );
  }

  _header() {
    return const Column(
      children: [
        Text(
          "Welcome Back",
          style: TextStyle(fontSize: 40, fontWeight: FontWeight.bold),
        ),
      ],
    );
  }

  _inputField(context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: [
        // Username TextField with a controller
        TextField(
          controller: _usernameController,  // Step 2: Assign the controller
          decoration: InputDecoration(
            hintText: "Username",
            border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(18),
              borderSide: BorderSide.none,
            ),
            fillColor: Theme.of(context).primaryColor.withAlpha(26),
            filled: true,
            prefixIcon: const Icon(Icons.person),
          ),
        ),
        const SizedBox(height: 10),
        // Password TextField with a controller
        TextField(
          controller: _passwordController,  // Step 2: Assign the controller
          decoration: InputDecoration(
            hintText: "Password",
            border: OutlineInputBorder(
              borderRadius: BorderRadius.circular(18),
              borderSide: BorderSide.none,
            ),
            fillColor: Theme.of(context).primaryColor.withAlpha(26),
            filled: true,
            prefixIcon: const Icon(Icons.lock),
          ),
          obscureText: true,
        ),
        const SizedBox(height: 50),
        // Login button that captures the input
        ElevatedButton(
          onPressed: () {
            _login(context);
          },
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.green,
            shape: const StadiumBorder(),
            padding: const EdgeInsets.symmetric(vertical: 16),
          ),
          child: const Text(
            "Login",
            style: TextStyle(fontSize: 20, color: Colors.white),
          ),
        ),
        const SizedBox(height: 20),
        Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            const Text("Don't have an account? "),
            GestureDetector(
              onTap: () {
                Navigator.pushNamed(context, '/register');
              },
              child: const Text(
                "Sign up",
                style: TextStyle(
                  color: Colors.red,
                  fontWeight: FontWeight.bold,
                ),
              ),
            ),
          ],
        ),
      ],
    );
  }

  Future<bool> _wasForceKilled() async {
    final snapshot = await FirebaseDatabase.instance.ref("data/users/lastActiveTime").get();
    final lastActiveTime = snapshot.value as int? ?? 0;
    final now = DateTime.now().millisecondsSinceEpoch;
    return (now - lastActiveTime) > (1000 * 60 * 60 * 10); // 10 tiếng
  }
  Future<void> _startKillMonitorService() async {
    try {
      await platform.invokeMethod('startService');
    } on PlatformException catch (e) {
      logger.e("Không thể khởi động service: ${e.code} - ${e.message}");
    }
  }


  // Step 3: Method to handle login and capture input data
  void _login(BuildContext context) async{
    String username = _usernameController.text;  // Get the username input
    String password = _passwordController.text;  // Get the password input
    final navigator = Navigator.of(context);
    final messenger = ScaffoldMessenger.of(context);

    DatabaseReference usersData = FirebaseDatabase.instance.ref();
    DataSnapshot usersSnapshot = await usersData.child("data/users").get();
    dynamic data;

    if (usersSnapshot.exists){
      data = usersSnapshot.value as Map<dynamic, dynamic>;
    } else{
      logger.e("usersSnapshot don't esixt");
      return;
    }
    // Map<String, dynamic> userData = snapshot.value as Map<String, dynamic>;
    logger.i("📦 Full data: $data");
    logger.i("👤 Nhập: $username | 🔑 $password");
    logger.i(data['number_of_users']);
    logger.i(data['using']);
    
    bool valid = false;
    await Future.delayed(const Duration(seconds: 1));

    bool canLogin = false;
    if (data['using'] == false) {
      canLogin = true;
    } else {
      canLogin = await _wasForceKilled();
    }

    if (canLogin) {
      for (int i = 1; i <= data['number_of_users']; i++) {
        if ((username == data['user$i']['username']) &&
            (password == data['user$i']['password'])) {
          valid = true;
          _startKillMonitorService(); // Khởi động Android service
          usersData.child("data/users/using").set(true);
          usersData.child("data/users/lastActiveTime").set(DateTime.now().millisecondsSinceEpoch);
          navigator.pushReplacementNamed('/home');
          break;
        }
      }
    }
    if (valid == false) {
      
      // If credentials are incorrect, show an error message
      messenger.showSnackBar(
        const SnackBar(
          content: Center(child: Text("Login failed")),
          backgroundColor: Colors.red,
          duration: Duration(seconds: 2),
        ),
      );
    }
  }
}
