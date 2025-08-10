import 'dart:async';
import 'package:flutter/material.dart';

class SplashScreen extends StatefulWidget {
  const SplashScreen({super.key});

  @override
  State<SplashScreen> createState() => _SplashScreen1State();
}

class _SplashScreen1State extends State<SplashScreen> {

  @override
  void initState() {
    initStartApp();
    super.initState();
  }

  void initStartApp() async {
    await Future.delayed(const Duration(seconds: 2), () {
      if (!mounted) return;
      Navigator.pushReplacementNamed(context, '/login');
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        body: Center(
            child: Container(
                height: MediaQuery.of(context).size.height,
                width: MediaQuery.of(context).size.width,
                // width: 300,
                // height: 300,
                decoration: const BoxDecoration(color: Colors.white),
                child: Center(
                    child:
                    Transform.scale(
                        scale: 1.0,
                        child: Image.asset(
                          "assets/splash_ros.png",
                        )
                    )
                )
            )

        )
    );
  }
}
