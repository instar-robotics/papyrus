let pkgs1 = import /home/nschoe/nixpkgs {};
    pkgs2 = import /home/nschoe/airapkgs {};
in
  pkgs1.stdenv.mkDerivation {
    name = "papyrus-dev-env";
    buildInputs = [ pkgs1.pkgconfig pkgs1.cryptopp pkgs1.qt5.qtbase pkgs1.qt5.qtsvg pkgs1.cmake pkgs2.ros_comm  ];
}




# stdenv.mkDerivation {
	# name = "papyrus-dev-environment";
  # buildInputs = [ pkgconfig cryptopp qt5.qtbase qt5.qtsvg cmake ];
  # buildInputs = [ pkgconfig cryptopp qt5.full cmake ];
# }
