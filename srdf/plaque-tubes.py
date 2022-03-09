handle_fmt="""
<handle name="handle_{i:02}" clearance="0.05">
    <position rpy="-1.57079 1.57079 0"/>
    <link name="hole_{i:02}_link" />
    <mask>1 1 1 1 1 1</mask>
</handle>"""

print('<robot name="plaque-tubes">')
for k in range(44):
    print(handle_fmt.format(i=k))
print('</robot>')