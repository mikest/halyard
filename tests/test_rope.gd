class_name GdUnitExampleTest
extends GdUnitTestSuite

func test_example():
 assert_str("This is a example message") \
   .has_length(25) \
   .starts_with("This is a ex")