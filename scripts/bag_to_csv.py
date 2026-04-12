#!/usr/bin/env python3
import sys
import csv
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def bag_to_csv(bag_path, output_dir=None):
    bag_path = Path(bag_path)

    if bag_path.suffix == '.db3':
        bag_dir = bag_path.parent
    else:
        bag_dir = bag_path

    if output_dir is None:
        output_dir = bag_dir / 'csv_output'
    else:
        output_dir = Path(output_dir)

    output_dir.mkdir(exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_type_map = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_type_map}

    topic_writers = {}

    print(f'Converting bag: {bag_dir}')
    print(f'Output directory: {output_dir}')
    print(f'Topics found: {len(type_map)}')

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic not in topic_writers:
            msg_type = get_message(type_map[topic])
            csv_file = output_dir / f"{topic.replace('/', '_')[1:]}.csv"
            topic_writers[topic] = {
                'file': open(csv_file, 'w', newline=''),
                'writer': None,
                'msg_type': msg_type
            }
            print(f'  {topic} -> {csv_file.name}')

        msg = deserialize_message(data, topic_writers[topic]['msg_type'])
        msg_dict = message_to_dict(msg, timestamp)

        if topic_writers[topic]['writer'] is None:
            topic_writers[topic]['writer'] = csv.DictWriter(
                topic_writers[topic]['file'],
                fieldnames=msg_dict.keys()
            )
            topic_writers[topic]['writer'].writeheader()

        topic_writers[topic]['writer'].writerow(msg_dict)

    for tw in topic_writers.values():
        tw['file'].close()

    print(f'\nConversion complete! Files saved to: {output_dir}')

def message_to_dict(msg, timestamp):
    result = {'timestamp': timestamp}

    for field in msg.get_fields_and_field_types():
        value = getattr(msg, field)

        if hasattr(value, 'get_fields_and_field_types'):
            for subfield in value.get_fields_and_field_types():
                result[f'{field}.{subfield}'] = getattr(value, subfield)
        elif isinstance(value, (list, tuple)):
            if len(value) > 0 and hasattr(value[0], 'get_fields_and_field_types'):
                for i, item in enumerate(value):
                    for subfield in item.get_fields_and_field_types():
                        result[f'{field}[{i}].{subfield}'] = getattr(item, subfield)
            else:
                result[field] = str(value)
        else:
            result[field] = value

    return result

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 bag_to_csv.py <bag_path> [output_dir]')
        sys.exit(1)

    bag_path = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else None

    bag_to_csv(bag_path, output_dir)
