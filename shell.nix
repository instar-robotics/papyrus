with import <nixpkgs> {};
stdenv.mkDerivation {
	name = "papyrus-dev-environment";
  buildInputs = [ pkgconfig cryptopp qt5.full qtcreator gdb cmake ];
}
