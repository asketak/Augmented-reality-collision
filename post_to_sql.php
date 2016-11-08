<?php

$size = $_POST["size"];
$object = $_POST["object"];
$box = $_POST["box"];
$data = $box . " " . $object . " " . $size
$fname = "data.txt";

file_put_contents($fname,$data);
print($likes_sql);
?>