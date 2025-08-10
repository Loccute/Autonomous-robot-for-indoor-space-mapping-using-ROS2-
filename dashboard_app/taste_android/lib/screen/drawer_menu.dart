import 'package:flutter/material.dart';
import 'package:firebase_database/firebase_database.dart';

const kTitle = 'MENU';

class DrawerMenu extends StatefulWidget {
  const DrawerMenu({super.key});

  @override
  State<DrawerMenu> createState() => _DrawerMenuState();
}

class _DrawerMenuState extends State<DrawerMenu> {
  @override
  Widget build(BuildContext context) {
    return Drawer(
      child: ListView(
        padding: EdgeInsets.zero,
        children: <Widget>[
          const DrawerHeader(
            decoration: BoxDecoration(
              color: Colors.teal,
            ),
            child: Center(
              child: Text(
                kTitle,
                style: TextStyle(
                  fontSize: 24,
                  color: Colors.white,
                ),
              ),
            ),
          ),
          getListTile('Home', onTap: () {
            Navigator.popAndPushNamed(context, '/home');
          }),
          getLine(),
          getListTile('Teleop', onTap: () {
            Navigator.popAndPushNamed(context, '/mapping');
          }),
          getLine(),
          getListTile('Setting', onTap: () {
            Navigator.popAndPushNamed(context, '/setting');
          }),
          getLine(),
          getListTile('Logout', onTap: () {
           
            _logout();
          }),
        ],
      ),
    );
  }

  Widget getLine() {
    return SizedBox(
      height: 0.5,
      child: Container(
        color: Colors.grey,
      ),
    );
  }

  Widget getListTile(title, {void Function()? onTap}) {
    return ListTile(
      title: Text(title),
      onTap: onTap,
    );
  }
  Future<void> _logout() async {
    final ref = FirebaseDatabase.instance.ref("data/users/using");

    try {
      await ref.set(false);
      await FirebaseDatabase.instance.ref("data/users/lastActiveTime").set(DateTime.now().millisecondsSinceEpoch);
      if (!mounted) return;
      Navigator.pushReplacementNamed(context, '/splash');
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(content: Text("Lỗi khi đăng xuất: $e")),
      );
  }
}
}


