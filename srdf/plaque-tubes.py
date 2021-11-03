handle_fmt="""
<handle name="handle_{i}" clearance="0.05">
    <position rpy="0 0 0"/>
    <link name="hole_{j:02}_link" />
    <mask>1 1 1 0 1 1</mask>
</handle>"""

print('<robot name="plaque-tubes">')
for i in range(44):
    print(handle_fmt.format(i=i, j=i+1))
print('</robot>')