import 'package:flutter/material.dart';
import 'package:flutter_inset_box_shadow/flutter_inset_box_shadow.dart'
    as inset_box_shadow;

class LedButton extends StatefulWidget {
  final VoidCallback onPressed;
  final bool isOn;
  final String channel;

  const LedButton({
    super.key,
    required this.onPressed,
    required this.isOn,
    required this.channel,
  });

  @override
  State<LedButton> createState() => _LedButtonState();
}

class _LedButtonState extends State<LedButton> {
  bool isPressed = false;

  @override
  Widget build(BuildContext context) {
    Offset distance = isPressed ? const Offset(6, 6) : const Offset(3, 3);
    double blur = isPressed ? 4.0 : 10.0;

    final String buttonText = widget.isOn
        ? 'Turn OFF ${widget.channel} LED'
        : 'Turn ON ${widget.channel} LED';

    return GestureDetector(
      onTapDown: (_) {
        setState(() {
          isPressed = true;
        });
      },
      onTapUp: (_) {
        setState(() {
          isPressed = false;
        });
      },
      onTapCancel: () {
        setState(() {
          isPressed = false;
        });
      },
      onTap: widget.onPressed,
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
              inset: widget.isOn,
            ),
            inset_box_shadow.BoxShadow(
              color: Colors.white,
              offset: -distance,
              blurRadius: blur,
              spreadRadius: 1,
              inset: widget.isOn,
            ),
          ],
        ),
        child: Center(
          child: Text(
            buttonText,
            style: TextStyle(
              fontSize: 16,
              color: widget.isOn ? Colors.red : Colors.green,
            ),
          ),
        ),
      ),
    );
  }
}
