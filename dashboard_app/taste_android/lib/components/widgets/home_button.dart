import 'package:flutter/material.dart';
import 'package:flutter_svg/flutter_svg.dart';
import 'package:flutter_inset_box_shadow/flutter_inset_box_shadow.dart' as inset_box_shadow;

class HomeButton extends StatefulWidget {
  final VoidCallback onPressed;

  const HomeButton({super.key, required this.onPressed});

  @override
  State<HomeButton> createState() => _HomeButtonState();
}

class _HomeButtonState extends State<HomeButton> {
  bool isPressed = false;

  @override
  Widget build(BuildContext context) {
    Offset distance = isPressed ? const Offset(6, 6) : const Offset(3, 3);
    double blur = isPressed ? 4.0 : 10.0;

    return GestureDetector(
      onTap: () {
        setState(() {
          isPressed = true;
        });
        Future.delayed(const Duration(milliseconds: 500), () {
          setState(() {
            isPressed = false;
          });
        });
        widget.onPressed();
      },
      child: AnimatedContainer(
        duration: const Duration(milliseconds: 200),
        padding: const EdgeInsets.symmetric(vertical: 4.0, horizontal: 6.0),
        decoration: inset_box_shadow.BoxDecoration(
          color: const Color.fromARGB(255, 237, 243, 250),
          borderRadius: BorderRadius.circular(8),
          boxShadow: [
            inset_box_shadow.BoxShadow(
              color: Colors.grey[500]!,
              offset: distance,
              blurRadius: blur,
              spreadRadius: 1,
              inset: isPressed,
            ),
            inset_box_shadow.BoxShadow(
              color: Colors.white,
              offset: -distance,
              blurRadius: blur,
              spreadRadius: 1,
              inset: isPressed,
            ),
          ],
        ),
        child: Center(
          child: SvgPicture.asset(
            'assets/button_images/home_button.svg',
            width: MediaQuery.of(context).size.width * 0.09,
            height: MediaQuery.of(context).size.height * 0.09,
          ),
        ),
      ),
    );
  }
}
