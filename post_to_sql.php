<?php

$size = $_POST["size"];
$object = $_POST["object"];
$box = $_POST["box"];
$data = $box . ":" . $object . ":" . $size;
$data2 = "huhu";
$fname = "data.txt";

$file = fopen($fname,"w");
echo fwrite($file,$data);
fclose($file);
?>