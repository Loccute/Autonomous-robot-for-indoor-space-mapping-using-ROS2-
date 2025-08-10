import 'package:flutter/material.dart';
import 'package:flutter_inset_box_shadow/flutter_inset_box_shadow.dart' as inset_box_shadow;

class NeumorphicButton extends StatefulWidget {
  final VoidCallback onPressed;
  final String buttonText;
  final Color buttonColor;
  final double height;
  final double width;

  const NeumorphicButton({
    super.key,
    required this.onPressed,
    required this.buttonText,
    required this.buttonColor,
    required this.height,
    required this.width,
  });

  @override
  State<NeumorphicButton> createState() => _NeumorphicButtonState();
}

class _NeumorphicButtonState extends State<NeumorphicButton> {
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
        height: widget.height,
        width: widget.width,
        padding: const EdgeInsets.symmetric(vertical: 4.0, horizontal: 6.0),
        decoration: inset_box_shadow.BoxDecoration(
          color: widget.buttonColor,
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
          child: Text(
            widget.buttonText,
            style: TextStyle(
              fontSize: MediaQuery.of(context).size.width * 0.02,
              color: Colors.black,
              fontWeight: FontWeight.bold,
            ),
          ),
        ),
      ),
    );
  }
}
