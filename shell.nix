with import <nixpkgs> {};
stdenv.mkDerivation {
	name = "papyrus-dev-environment";
	buildInputs = [ cryptopp ];
}
